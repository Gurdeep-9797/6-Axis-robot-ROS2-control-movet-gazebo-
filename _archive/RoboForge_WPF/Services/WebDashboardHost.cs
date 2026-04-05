using System;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using RoboForge_WPF.ViewModels;

namespace RoboForge_WPF.Services
{
    public class WebDashboardHost : IDisposable
    {
        private WebApplication? _app;
        private readonly MainViewModel _viewModel;
        private readonly int _port;

        public WebDashboardHost(MainViewModel viewModel, int port = 5050)
        {
            _viewModel = viewModel ?? throw new ArgumentNullException(nameof(viewModel));
            _port = port;
        }

        public async Task StartAsync()
        {
            if (_app != null) return;

            var builder = WebApplication.CreateBuilder();

            // Set up static files from wwwroot
            builder.WebHost.UseWebRoot("wwwroot");

            // Allow CORS just in case
            builder.Services.AddCors(options =>
            {
                options.AddDefaultPolicy(plc =>
                {
                    plc.AllowAnyOrigin().AllowAnyHeader().AllowAnyMethod();
                });
            });

            _app = builder.Build();

            _app.UseCors();
            _app.UseDefaultFiles();
            _app.UseStaticFiles();

            // API: Get State
            _app.MapGet("/api/state", () =>
            {
                var state = _viewModel.CurrentState;
                if (state == null)
                {
                    return Results.Json(new { status = "Disconnected" });
                }

                return Results.Json(new
                {
                    status = _viewModel.StatusText,
                    isRunning = _viewModel.IsRunning,
                    j1 = state.J1,
                    j2 = state.J2,
                    j3 = state.J3,
                    j4 = state.J4,
                    j5 = state.J5,
                    j6 = state.J6
                });
            });

            // API: Trigger Run/Pause
            _app.MapPost("/api/action/{command}", (string command) =>
            {
                // Marshal to UI thread
                System.Windows.Application.Current.Dispatcher.Invoke(() =>
                {
                    if (command == "run") _viewModel.IsRunning = true;
                    if (command == "pause") _viewModel.IsRunning = false;
                });
                return Results.Ok(new { success = true });
            });

            // API: Set Joints
            _app.MapPost("/api/joints", (double j1, double j2, double j3, double j4, double j5, double j6) =>
            {
                System.Windows.Application.Current.Dispatcher.Invoke(() =>
                {
                    if (_viewModel.CurrentState != null)
                    {
                        _viewModel.CurrentState.J1 = j1;
                        _viewModel.CurrentState.J2 = j2;
                        _viewModel.CurrentState.J3 = j3;
                        _viewModel.CurrentState.J4 = j4;
                        _viewModel.CurrentState.J5 = j5;
                        _viewModel.CurrentState.J6 = j6;
                    }
                });
                return Results.Ok(new { success = true });
            });

            _app.Urls.Add($"http://localhost:{_port}");

            try
            {
                await _app.StartAsync();
                LoggingService.Instance.Log($"Web Dashboard started on http://localhost:{_port}");
            }
            catch (Exception ex)
            {
                LoggingService.Instance.Log($"Failed to start Web Dashboard: {ex.Message}", "Error");
            }
        }

        public async Task StopAsync()
        {
            if (_app != null)
            {
                await _app.StopAsync();
                await _app.DisposeAsync();
                _app = null;
            }
        }

        public void Dispose()
        {
            StopAsync().GetAwaiter().GetResult();
        }
    }
}
