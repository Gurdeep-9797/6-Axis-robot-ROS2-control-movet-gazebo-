using System;
using System.IO;
using System.Collections.Generic;
using Microsoft.Data.Sqlite;
using RoboForge_WPF.Models;

namespace RoboForge_WPF.Services
{
    public class DatabaseManager
    {
        private readonly string _dbPath;

        public DatabaseManager()
        {
            // Store the database inside the isolated AppData location
            string appData = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
            string roboforgeFolder = Path.Combine(appData, "RoboForge");
            Directory.CreateDirectory(roboforgeFolder);
            
            _dbPath = Path.Combine(roboforgeFolder, "roboforge.db");
            InitializeDatabase();
        }

        private void InitializeDatabase()
        {
            using var connection = new SqliteConnection($"Data Source={_dbPath}");
            connection.Open();

            var command = connection.CreateCommand();
            
            // Note: Creating the tables if they don't exist
            command.CommandText = @"
                CREATE TABLE IF NOT EXISTS Workspaces (
                    Id INTEGER PRIMARY KEY AUTOINCREMENT,
                    Name TEXT NOT NULL,
                    Path TEXT NOT NULL,
                    LastModified DATETIME DEFAULT CURRENT_TIMESTAMP
                );

                CREATE TABLE IF NOT EXISTS RobotConfigs (
                    Id INTEGER PRIMARY KEY AUTOINCREMENT,
                    Name TEXT NOT NULL,
                    UrdfPath TEXT NOT NULL,
                    MaxSpeed REAL DEFAULT 1.0,
                    IpAddress TEXT DEFAULT '127.0.0.1',
                    Port INTEGER DEFAULT 5000,
                    LastUsed DATETIME DEFAULT CURRENT_TIMESTAMP
                );

                CREATE TABLE IF NOT EXISTS Programs (
                    Id INTEGER PRIMARY KEY AUTOINCREMENT,
                    WorkspaceId INTEGER NOT NULL,
                    Name TEXT NOT NULL,
                    Path TEXT NOT NULL,
                    LastRun DATETIME DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY(WorkspaceId) REFERENCES Workspaces(Id)
                );
            ";
            command.ExecuteNonQuery();

            // Seed with an initial Workspace if empty
            if (GetRecentWorkspaces().Count == 0)
            {
                SaveWorkspace(new WorkspaceModel {
                    Name = "Initial Workspace",
                    Path = "C:\\RoboForge\\Workspaces\\Default",
                    LastModified = DateTime.Now
                });
            }

            // Seed with an initial RobotConfig if empty
            if (GetRecentRobotConfigs().Count == 0)
            {
                SaveRobotConfig(new RobotConfigModel {
                    Name = "Default 6-Axis (Gazebo)",
                    UrdfPath = "robot.urdf",
                    IpAddress = "127.0.0.1",
                    Port = 5000,
                    MaxSpeed = 0.5,
                    LastUsed = DateTime.Now
                });
            }
        }

        // --- Workspaces ---
        public List<WorkspaceModel> GetRecentWorkspaces()
        {
            var list = new List<WorkspaceModel>();
            using var connection = new SqliteConnection($"Data Source={_dbPath}");
            connection.Open();

            var command = connection.CreateCommand();
            command.CommandText = "SELECT Id, Name, Path, LastModified FROM Workspaces ORDER BY LastModified DESC LIMIT 10;";
            
            using var reader = command.ExecuteReader();
            while (reader.Read())
            {
                list.Add(new WorkspaceModel {
                    Id = reader.GetInt32(0),
                    Name = reader.GetString(1),
                    Path = reader.GetString(2),
                    LastModified = reader.GetDateTime(3)
                });
            }
            return list;
        }

        public void SaveWorkspace(WorkspaceModel workspace)
        {
            using var connection = new SqliteConnection($"Data Source={_dbPath}");
            connection.Open();

            var command = connection.CreateCommand();
            if (workspace.Id > 0)
            {
                command.CommandText = "UPDATE Workspaces SET Name = $name, Path = $path, LastModified = CURRENT_TIMESTAMP WHERE Id = $id;";
                command.Parameters.AddWithValue("$id", workspace.Id);
            }
            else
            {
                command.CommandText = "INSERT INTO Workspaces (Name, Path, LastModified) VALUES ($name, $path, CURRENT_TIMESTAMP);";
            }
            command.Parameters.AddWithValue("$name", workspace.Name);
            command.Parameters.AddWithValue("$path", workspace.Path);
            command.ExecuteNonQuery();
        }

        // --- Robot Configs ---
        public List<RobotConfigModel> GetRecentRobotConfigs()
        {
            var list = new List<RobotConfigModel>();
            using var connection = new SqliteConnection($"Data Source={_dbPath}");
            connection.Open();

            var command = connection.CreateCommand();
            command.CommandText = "SELECT Id, Name, UrdfPath, MaxSpeed, IpAddress, Port, LastUsed FROM RobotConfigs ORDER BY LastUsed DESC LIMIT 10;";
            
            using var reader = command.ExecuteReader();
            while (reader.Read())
            {
                list.Add(new RobotConfigModel {
                    Id = reader.GetInt32(0),
                    Name = reader.GetString(1),
                    UrdfPath = reader.GetString(2),
                    MaxSpeed = reader.GetDouble(3),
                    IpAddress = reader.GetString(4),
                    Port = reader.GetInt32(5),
                    LastUsed = reader.GetDateTime(6)
                });
            }
            return list;
        }

        public void SaveRobotConfig(RobotConfigModel robot)
        {
            using var connection = new SqliteConnection($"Data Source={_dbPath}");
            connection.Open();

            var command = connection.CreateCommand();
            if (robot.Id > 0)
            {
                command.CommandText = "UPDATE RobotConfigs SET Name = $name, UrdfPath = $urdf, MaxSpeed = $speed, IpAddress = $ip, Port = $port, LastUsed = CURRENT_TIMESTAMP WHERE Id = $id;";
                command.Parameters.AddWithValue("$id", robot.Id);
            }
            else
            {
                command.CommandText = "INSERT INTO RobotConfigs (Name, UrdfPath, MaxSpeed, IpAddress, Port, LastUsed) VALUES ($name, $urdf, $speed, $ip, $port, CURRENT_TIMESTAMP);";
            }
            command.Parameters.AddWithValue("$name", robot.Name);
            command.Parameters.AddWithValue("$urdf", robot.UrdfPath);
            command.Parameters.AddWithValue("$speed", robot.MaxSpeed);
            command.Parameters.AddWithValue("$ip", robot.IpAddress);
            command.Parameters.AddWithValue("$port", robot.Port);
            command.ExecuteNonQuery();
        }
    }
}
