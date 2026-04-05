using System;

namespace RoboForge.Domain
{
    public interface IEvent { }

    public record RobotStateChangedEvent(string RobotId, double[] Joints, Pose TCP) : IEvent;
    public record WaypointSelectedEvent(string WaypointId) : IEvent;
    public record ProgramNodeExecutingEvent(string NodeId) : IEvent;
    public record BackendConnectedEvent(bool IsConnected) : IEvent;
    public record IKSolvedEvent(string WaypointId, double[] Joints) : IEvent;
}
