using System;
using System.Windows;

namespace RoboForge_WPF
{
    public class LogEntry
    {
        public string Timestamp { get; set; } = DateTime.Now.ToString("HH:mm:ss");
        public string Message { get; set; } = "";
        public string Severity { get; set; } = "Info"; // Info, Success, Warning, Error
    }
}
