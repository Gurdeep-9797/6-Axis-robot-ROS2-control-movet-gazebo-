using System;
using System.Collections.Concurrent;
using System.Collections.ObjectModel;
using System.Windows.Threading;

namespace RoboForge_WPF.Services
{
    public class LoggingService
    {
        private static LoggingService? _instance;
        public static LoggingService Instance => _instance ??= new LoggingService();

        public ObservableCollection<LogEntry> Logs { get; } = new ObservableCollection<LogEntry>();
        private readonly ConcurrentQueue<LogEntry> _logQueue = new ConcurrentQueue<LogEntry>();
        private readonly DispatcherTimer _timer;

        public event Action? LogsUpdated;

        private LoggingService()
        {
            _timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(50)
            };
            _timer.Tick += (s, e) => ProcessQueue();
            _timer.Start();
        }

        public void ProcessQueue()
        {
            if (_logQueue.IsEmpty) return;

            bool added = false;
            while (_logQueue.TryDequeue(out var entry))
            {
                Logs.Add(entry);
                added = true;
            }

            if (added)
            {
                LogsUpdated?.Invoke();
            }
        }

        public void Log(string message, string severity = "Info")
        {
            _logQueue.Enqueue(new LogEntry
            {
                Timestamp = DateTime.Now.ToString("HH:mm:ss"),
                Message = message,
                Severity = severity
            });
        }
    }
}
