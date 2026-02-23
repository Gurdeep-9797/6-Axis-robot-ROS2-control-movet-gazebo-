using System.Threading.Tasks;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.Services
{
    public interface IRobotDriver
    {
        void Connect();
        void Disconnect();
        
        bool IsConnected { get; }

        RobotState GetCurrentState();

        Task SendJointTorques(double[] torques);
        Task SendJointPositions(double[] anglesDeg);
        Task ExecuteTrajectoryPoint(double[] anglesDeg, double timeFromStartSec);
        
        // Setup a subscription to receive async updates
        event System.Action<RobotState> StateUpdated;
    }
}
