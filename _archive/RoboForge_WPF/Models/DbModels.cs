using System;

namespace RoboForge_WPF.Models
{
    public class WorkspaceModel
    {
        public int Id { get; set; }
        public string Name { get; set; } = "New Workspace";
        public string Path { get; set; } = "";
        public DateTime LastModified { get; set; } = DateTime.Now;
    }

    public class RobotConfigModel
    {
        public int Id { get; set; }
        public string Name { get; set; } = "Industrial 6-DoF";
        public string UrdfPath { get; set; } = "";
        public double MaxSpeed { get; set; } = 1.0;
        public string IpAddress { get; set; } = "127.0.0.1";
        public int Port { get; set; } = 5000;
        public DateTime LastUsed { get; set; } = DateTime.Now;
    }

    public class ProgramFileModel
    {
        public int Id { get; set; }
        public int WorkspaceId { get; set; }
        public string Name { get; set; } = "Main Routine";
        public string Path { get; set; } = "";
        public DateTime LastRun { get; set; } = DateTime.Now;
    }
}
