using System.Collections.Concurrent;

namespace TeachPendant_WPF.Services
{
    public class WaypointStore
    {
        private static WaypointStore? _instance;
        public static WaypointStore Instance => _instance ??= new WaypointStore();

        // Map Waypoint ID to Joint Angles [J1..J6]
        private readonly ConcurrentDictionary<string, double[]> _waypoints = new ConcurrentDictionary<string, double[]>();

        private WaypointStore()
        {
            // Add some default placeholder waypoints to match defaults in MainWindow
            _waypoints["WP_Home"] = new double[] { 0, 0, 0, 0, 0, 0 };
            _waypoints["WP_Approach"] = new double[] { 45, 10, -20, 0, 10, 0 };
            _waypoints["WP_Pick"] = new double[] { 45, 30, -45, 0, 15, 0 };
            _waypoints["WP_Place"] = new double[] { -45, 30, -45, 0, 15, 0 };
        }

        public void AddOrUpdateWaypoint(string name, double[] joints)
        {
            _waypoints[name] = (double[])joints.Clone();
        }

        public double[]? GetWaypoint(string name)
        {
            if (_waypoints.TryGetValue(name, out var joints))
            {
                return (double[])joints.Clone();
            }
            return null;
        }

        public bool RemoveWaypoint(string name)
        {
            return _waypoints.TryRemove(name, out _);
        }

        public string[] GetAllWaypointNames()
        {
            return [.. _waypoints.Keys];
        }
    }
}
