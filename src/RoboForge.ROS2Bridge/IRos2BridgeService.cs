using System.Threading;
using System.Threading.Tasks;
using RoboForge.Domain;

// In a real project with rcldotnet, these would be generated messages.
// We are defining stubs here to allow the project structure to compile.
namespace RoboForge.ROS2Bridge.Messages
{
    public class MoveGroupSequenceActionGoal { }
    public class MoveGroupActionGoal { }
    public class JointState 
    { 
        public double[] Position { get; set; } 
        public double[] Velocity { get; set; } 
    }
}

namespace RoboForge.ROS2Bridge
{
    public interface IRos2BridgeService
    {
        Task<IKResult> SolveIKAsync(Pose target, CancellationToken ct);
        Task<MoveResult> SendMoveGroupGoalAsync(Messages.MoveGroupSequenceActionGoal goal, CancellationToken ct);
        Task PublishIOStateAsync(int channel, bool value);
    }
}
