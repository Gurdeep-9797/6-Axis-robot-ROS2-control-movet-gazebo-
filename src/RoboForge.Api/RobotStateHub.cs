using Microsoft.AspNetCore.SignalR;

namespace RoboForge.Api.Hubs
{
    // SignalR hub for real-time streaming to the WPF UI
    public class RobotStateHub : Hub
    {
        // Clients connect to this hub and receive:
        // "RobotStateUpdate" messages from the ROS2Bridge Service.
        
        // Also used for Console log streaming
        public async Task SendConsoleLog(string level, string message)
        {
            await Clients.All.SendAsync("ReceiveLog", level, message);
        }
    }
}
