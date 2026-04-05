using System;
using System.Threading;
using System.Threading.Tasks;
using RoboForge.Domain;
using RoboForge.ROS2Bridge;
using RoboForge.Application;

namespace RoboForge.Execution
{
    public interface IEventBus
    {
        void Publish<T>(T message) where T : IEvent;
        void Subscribe<T>(Action<T> handler) where T : IEvent;
    }

    public interface IIOController
    {
        Task SetDOAsync(int channel, bool value, CancellationToken ct);
    }

    public class ExecutionException : Exception
    {
        public string NodeId { get; }
        public ExecutionException(string error, string nodeId) : base($"Execution failed for node {nodeId}: {error}")
        {
            NodeId = nodeId;
        }
    }

    public class ExecutionEngine
    {
        private readonly IEventBus _eventBus;
        private readonly IRos2BridgeService _ros2Bridge;
        private readonly IRToRos2GoalTranslator _translator;
        private readonly IIOController _ioController;
        private double _speedOverride = 1.0;

        public ExecutionEngine(
            IEventBus eventBus,
            IRos2BridgeService ros2Bridge,
            IRToRos2GoalTranslator translator,
            IIOController ioController)
        {
            _eventBus = eventBus;
            _ros2Bridge = ros2Bridge;
            _translator = translator;
            _ioController = ioController;
        }

        public async Task StartAsync(ProgramNode program, CancellationToken ct)
        {
            try
            {
                // Execution scheduler logic
                foreach (var procedure in program.Procedures)
                {
                    await ExecuteNodeAsync(procedure, ct);
                }
            }
            catch (OperationCanceledException)
            {
                // Stopped normally
            }
        }

        private async Task ExecuteNodeAsync(IRNode node, CancellationToken ct)
        {
            _eventBus.Publish(new ProgramNodeExecutingEvent(node.Id));

            switch (node)
            {
                case MoveJNode j:
                case MoveLNode l:
                case MoveCNode c:
                    var goal = _translator.Translate(new[] { node }, _speedOverride);
                    var result = await _ros2Bridge.SendMoveGroupGoalAsync(goal, ct);
                    if (!result.Success) throw new ExecutionException(result.ErrorCode, node.Id);
                    break;
                case WaitNode w:
                    await Task.Delay(TimeSpan.FromSeconds(w.DurationSeconds), ct);
                    break;
                case SetDONode d:
                    await _ioController.SetDOAsync(d.Channel, d.Value, ct);
                    await _ros2Bridge.PublishIOStateAsync(d.Channel, d.Value);
                    break;
                case WhileNode w:
                    while (!ct.IsCancellationRequested)
                    {
                        // Simplified while true, in reality evaluates w.Condition
                        foreach (var child in w.Body)
                        {
                            await ExecuteNodeAsync(child, ct);
                        }
                    }
                    break;
                case ProcNode p:
                    foreach (var child in p.Body)
                    {
                        await ExecuteNodeAsync(child, ct);
                    }
                    break;
            }
        }
    }
}
