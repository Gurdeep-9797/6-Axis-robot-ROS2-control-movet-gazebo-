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
        
        // Wait for physical feedback to confirm position reached
        Task<bool> WaitForJointStateAsync(double[] targetAngles, double toleranceDeg, int timeoutMs = 10000);
        
        // Setup a subscription to receive async updates
        event System.Action<RobotState> StateUpdated;
    }
}
