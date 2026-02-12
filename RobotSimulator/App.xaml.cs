using System;
using System.IO;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Threading;

namespace RobotSimulator
{
    /// <summary>
    /// Application entry point with MANDATORY global exception handling.
    /// All exceptions MUST be visible - no silent exits allowed.
    /// </summary>
    public partial class App : Application
    {
        private static readonly string LogPath = Path.Combine(
            AppDomain.CurrentDomain.BaseDirectory, "simulator_crash.log");

        public App()
        {
            // STEP 1: Force all exceptions to be visible
            
            // Dispatcher (UI thread) exceptions
            DispatcherUnhandledException += App_DispatcherUnhandledException;
            
            // AppDomain exceptions (non-UI threads)
            AppDomain.CurrentDomain.UnhandledException += CurrentDomain_UnhandledException;
            
            // Task exceptions
            TaskScheduler.UnobservedTaskException += TaskScheduler_UnobservedTaskException;
            
            // Ensure app doesn't close unexpectedly
            ShutdownMode = ShutdownMode.OnMainWindowClose;
            
            LogMessage("=== APPLICATION STARTING ===");
            LogMessage($"Runtime: {Environment.Version}");
            LogMessage($"OS: {Environment.OSVersion}");
            LogMessage($"64-bit: {Environment.Is64BitProcess}");
        }

        private void App_DispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            string message = FormatException("DISPATCHER EXCEPTION", e.Exception);
            LogMessage(message);
            ShowErrorAndLog(message);
            e.Handled = true; // Prevent silent exit - keep app alive
        }

        private void CurrentDomain_UnhandledException(object sender, UnhandledExceptionEventArgs e)
        {
            var ex = e.ExceptionObject as Exception;
            string message = FormatException("APPDOMAIN EXCEPTION", ex);
            LogMessage(message);
            ShowErrorAndLog(message);
        }

        private void TaskScheduler_UnobservedTaskException(object? sender, UnobservedTaskExceptionEventArgs e)
        {
            string message = FormatException("TASK EXCEPTION", e.Exception);
            LogMessage(message);
            ShowErrorAndLog(message);
            e.SetObserved(); // Prevent crash
        }

        private static string FormatException(string source, Exception? ex)
        {
            if (ex == null) return $"[{source}] Unknown exception (null)";
            
            return $"""
                ========================================
                [{source}] {DateTime.Now:yyyy-MM-dd HH:mm:ss}
                ----------------------------------------
                Type: {ex.GetType().FullName}
                Message: {ex.Message}
                ----------------------------------------
                Stack Trace:
                {ex.StackTrace}
                ----------------------------------------
                Inner Exception: {ex.InnerException?.Message ?? "None"}
                ========================================
                """;
        }

        private static void ShowErrorAndLog(string message)
        {
            try
            {
                MessageBox.Show(message, "Robot Simulator - CRASH DETECTED", 
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
            catch
            {
                // If MessageBox fails, at least we have the log
            }
        }

        public static void LogMessage(string message)
        {
            try
            {
                string logLine = $"[{DateTime.Now:HH:mm:ss.fff}] {message}\n";
                File.AppendAllText(LogPath, logLine);
                Console.WriteLine(logLine);
            }
            catch
            {
                // Logging should never crash the app
            }
        }
    }
}
