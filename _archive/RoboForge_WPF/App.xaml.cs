using System;
using System.IO;
using System.Threading.Tasks;
using System.Windows;

namespace RoboForge_WPF
{
    public partial class App : Application
    {
        private static readonly string CrashLogPath = Path.Combine(
            AppDomain.CurrentDomain.BaseDirectory, "roboforge_crash.log");

        protected override void OnStartup(StartupEventArgs e)
        {
            // 1. CLR-level fatal crash (thread exception)
            AppDomain.CurrentDomain.UnhandledException += (s, args) =>
            {
                var ex = args.ExceptionObject as Exception;
                WriteCrashLog("FATAL", ex);
                MessageBox.Show($"[FATAL] {ex?.Message}", "RoboForge Crash", MessageBoxButton.OK, MessageBoxImage.Error);
            };

            // 2. WPF Dispatcher (UI thread) unhandled exception
            DispatcherUnhandledException += (s, args) =>
            {
                WriteCrashLog("UI_ERROR", args.Exception);
                // Log to LoggingService if available, so it shows in the console
                try { Services.LoggingService.Instance.Log($"UI Exception: {args.Exception.Message}", "Error"); } catch { }
                MessageBox.Show($"[UI ERROR] {args.Exception.Message}", "RoboForge Error", MessageBoxButton.OK, MessageBoxImage.Error);
                args.Handled = true; // Prevent crash; keep app alive
            };

            // 3. Unobserved async Task exceptions (e.g., from ROS/WebSocket calls)
            TaskScheduler.UnobservedTaskException += (s, args) =>
            {
                WriteCrashLog("ASYNC_FAULT", args.Exception);
                try { Services.LoggingService.Instance.Log($"Async fault: {args.Exception?.InnerException?.Message ?? args.Exception?.Message}", "Error"); } catch { }
                args.SetObserved(); // Prevent process termination
            };

            base.OnStartup(e);
        }

        private static void WriteCrashLog(string severity, Exception? ex)
        {
            try
            {
                var entry = $"[{severity}] {DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}: {ex}{Environment.NewLine}";
                File.AppendAllText(CrashLogPath, entry);
            }
            catch { /* If we can't write the crash log, there's nothing else we can do */ }
        }
    }
}
