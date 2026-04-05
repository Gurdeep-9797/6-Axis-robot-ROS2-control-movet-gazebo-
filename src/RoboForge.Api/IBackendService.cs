using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using RoboForge.Domain;

namespace RoboForge.Api
{
    // This interface defines the contract between the WPF frontend and the API backend
    // In a full implementation, this is generated from a gRPC .proto file.
    public interface IBackendService
    {
        Task CompileAsync(string irJson);
        Task RunAsync();
        Task StopAsync();
        Task<IKResult> SolveIKAsync(Pose target);
        Task JogAsync(JogRequest req);
        Task<object> LoadRobotAsync(string modelId);
        IAsyncEnumerable<RobotState> StreamStateAsync(CancellationToken ct);
    }
}
