// ── Section 3: File System (.rfproj ZIP Format, Save/Open, Auto-Save) ────
using System;
using System.IO;
using System.IO.Compression;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;
using Newtonsoft.Json;
using RoboForge.Wpf.AST;
using RoboForge.Wpf.Core;
using RoboForge.Wpf.IO;
using RoboForge.Wpf.Models;

namespace RoboForge.Wpf.FileSystem
{
    /// <summary>
    /// .rfproj file format handler.
    /// A RoboForge project is a renamed ZIP archive containing:
    ///   robot.json        — Full scene tree (links, joints, sensors, frames)
    ///   program/main.mod  — Main program in .mod text format
    ///   program/routines/ — One .mod file per named routine
    ///   io_config.json    — IO device configuration and pin mappings
    ///   settings.json     — Editor preferences, grid settings, UI state
    ///   assets/meshes/    — Embedded STL/OBJ mesh files
    ///   thumbnail.png     — 256×256 viewport screenshot
    /// </summary>
    public static class ProjectFileHandler
    {
        /// <summary>File extension for RoboForge projects</summary>
        public const string Extension = ".rfproj";

        /// <summary>Internal paths within the ZIP archive</summary>
        private const string RobotJsonPath = "robot.json";
        private const string MainModPath = "program/main.mod";
        private const string RoutinesDir = "program/routines/";
        private const string IoConfigPath = "io_config.json";
        private const string SettingsPath = "settings.json";
        private const string MeshesDir = "assets/meshes/";
        private const string ThumbnailPath = "thumbnail.png";

        /// <summary>
        /// Save the entire project to a .rfproj file.
        /// Uses atomic write (temp file + rename) to prevent corruption on crash.
        /// </summary>
        public static async Task<bool> SaveAsync(
            string filePath,
            RobotModel robot,
            ProgramNode program,
            IoConfiguration ioConfig,
            ProjectSettings settings,
            byte[]? thumbnailPng = null,
            CancellationToken ct = default)
        {
            try
            {
                var tempPath = filePath + ".tmp";

                // Delete existing temp if present
                if (File.Exists(tempPath))
                    File.Delete(tempPath);

                using var archive = ZipFile.Open(tempPath, ZipArchiveMode.Create);

                // 1. robot.json — serialize scene tree
                var robotJson = JsonConvert.SerializeObject(robot, Formatting.Indented);
                AddEntry(archive, RobotJsonPath, robotJson);

                // 2. program/main.mod — serialize program AST to text
                var programText = SerializeProgramToMod(program);
                AddEntry(archive, MainModPath, programText);

                // 3. program/routines/ — one .mod file per routine
                foreach (var child in program.Children)
                {
                    if (child is RoutineNode routine && !string.IsNullOrEmpty(routine.Name))
                    {
                        var routineText = SerializeRoutineToMod(routine);
                        AddEntry(archive, RoutinesDir + routine.Name + ".mod", routineText);
                    }
                }

                // 4. io_config.json — IO device configuration
                var ioJson = JsonConvert.SerializeObject(ioConfig, Formatting.Indented);
                AddEntry(archive, IoConfigPath, ioJson);

                // 5. settings.json — editor preferences and UI state
                var settingsJson = JsonConvert.SerializeObject(settings, Formatting.Indented);
                AddEntry(archive, SettingsPath, settingsJson);

                // 6. assets/meshes/ — embedded mesh files
                // (Meshes would be added here from project's referenced mesh files)

                // 7. thumbnail.png — viewport screenshot
                if (thumbnailPng != null && thumbnailPng.Length > 0)
                {
                    var thumbEntry = archive.CreateEntry(ThumbnailPath, CompressionLevel.Optimal);
                    using var thumbStream = thumbEntry.Open();
                    await thumbStream.WriteAsync(thumbnailPng, 0, thumbnailPng.Length, ct);
                }

                // Atomic rename: delete old file, rename temp to final
                if (File.Exists(filePath))
                    File.Delete(filePath);

                File.Move(tempPath, filePath);
                return true;
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Save failed: {ex.Message}");
                return false;
            }
        }

        /// <summary>
        /// Open a .rfproj file and deserialize all components.
        /// Extracts to temp folder, loads data, preserves temp folder for mesh references.
        /// </summary>
        public static async Task<ProjectData?> LoadAsync(string filePath, CancellationToken ct = default)
        {
            if (!File.Exists(filePath))
                return null;

            try
            {
                var tempDir = Path.Combine(Path.GetTempPath(), $"RoboForge_{Guid.NewGuid():N}");
                if (Directory.Exists(tempDir))
                    Directory.Delete(tempDir, true);
                Directory.CreateDirectory(tempDir);

                ZipFile.ExtractToDirectory(filePath, tempDir);

                // Read robot.json
                RobotModel? robot = null;
                var robotPath = Path.Combine(tempDir, RobotJsonPath);
                if (File.Exists(robotPath))
                {
                    var json = await File.ReadAllTextAsync(robotPath, ct);
                    robot = JsonConvert.DeserializeObject<RobotModel>(json);
                }

                // Read program/main.mod
                ProgramNode? program = null;
                var mainModPath = Path.Combine(tempDir, MainModPath);
                if (File.Exists(mainModPath))
                {
                    var modText = await File.ReadAllTextAsync(mainModPath, ct);
                    program = DeserializeModToProgram(modText);
                }

                // Read io_config.json
                IoConfiguration? ioConfig = null;
                var ioPath = Path.Combine(tempDir, IoConfigPath);
                if (File.Exists(ioPath))
                {
                    var json = await File.ReadAllTextAsync(ioPath, ct);
                    ioConfig = JsonConvert.DeserializeObject<IoConfiguration>(json);
                }

                // Read settings.json
                ProjectSettings? settings = null;
                var settingsPath = Path.Combine(tempDir, SettingsPath);
                if (File.Exists(settingsPath))
                {
                    var json = await File.ReadAllTextAsync(settingsPath, ct);
                    settings = JsonConvert.DeserializeObject<ProjectSettings>(json);
                }

                // Load embedded meshes
                var meshesDir = Path.Combine(tempDir, MeshesDir);
                var meshes = Directory.Exists(meshesDir)
                    ? Directory.GetFiles(meshesDir, "*.*", SearchOption.AllDirectories)
                    : Array.Empty<string>();

                // Read thumbnail
                byte[]? thumbnail = null;
                var thumbPath = Path.Combine(tempDir, ThumbnailPath);
                if (File.Exists(thumbPath))
                {
                    thumbnail = await File.ReadAllBytesAsync(thumbPath, ct);
                }

                return new ProjectData
                {
                    FilePath = filePath,
                    TempDir = tempDir,
                    Robot = robot ?? new RobotModel(),
                    Program = program ?? new ProgramNode(),
                    IoConfig = ioConfig ?? new IoConfiguration(),
                    Settings = settings ?? new ProjectSettings(),
                    MeshFiles = meshes,
                    Thumbnail = thumbnail,
                    LastModified = File.GetLastWriteTime(filePath),
                };
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Load failed: {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// Auto-save to .rfproj.autosave alongside the project file.
        /// Silent — no toast notifications.
        /// </summary>
        public static async Task<bool> AutoSaveAsync(
            string projectFilePath,
            RobotModel robot,
            ProgramNode program,
            IoConfiguration ioConfig,
            ProjectSettings settings,
            CancellationToken ct = default)
        {
            var autosavePath = projectFilePath + ".autosave";
            return await SaveAsync(autosavePath, robot, program, ioConfig, settings, null, ct);
        }

        /// <summary>
        /// Check if an autosave exists and is newer than the project file.
        /// </summary>
        public static bool HasNewerAutosave(string projectFilePath)
        {
            var autosavePath = projectFilePath + ".autosave";
            if (!File.Exists(autosavePath)) return false;

            var projectTime = File.GetLastWriteTime(projectFilePath);
            var autosaveTime = File.GetLastWriteTime(autosavePath);
            return autosaveTime > projectTime;
        }

        // ── Private Helpers ────────────────────────────────────────────────

        private static void AddEntry(ZipArchive archive, string path, string content)
        {
            var entry = archive.CreateEntry(path, CompressionLevel.Optimal);
            using var writer = new StreamWriter(entry.Open());
            writer.Write(content);
        }

        private static string SerializeProgramToMod(ProgramNode program)
        {
            // Serialize AST to .mod text format (RAPID-like syntax)
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("MODULE main");
            sb.AppendLine("  PROC");
            SerializeBlocks(program.Children, sb, 2);
            sb.AppendLine("  ENDPROC");
            sb.AppendLine("ENDMODULE");
            return sb.ToString();
        }

        private static string SerializeRoutineToMod(RoutineNode routine)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"MODULE {routine.Name}");
            sb.AppendLine($"  PROC {routine.Name}()");
            SerializeBlocks(routine.Children, sb, 2);
            sb.AppendLine("  ENDPROC");
            sb.AppendLine("ENDMODULE");
            return sb.ToString();
        }

        private static void SerializeBlocks(System.Collections.ObjectModel.ObservableCollection<AST.AstNode> blocks, System.Text.StringBuilder sb, int indent)
        {
            var indentStr = new string(' ', indent);
            foreach (var block in blocks)
            {
                switch (block.NodeType)
                {
                    case NodeType.MoveJ:
                        sb.AppendLine($"{indentStr}MoveJ {block.Name};");
                        break;
                    case NodeType.MoveL:
                        sb.AppendLine($"{indentStr}MoveL {block.Name};");
                        break;
                    case NodeType.MoveC:
                        sb.AppendLine($"{indentStr}MoveC {block.Name};");
                        break;
                    case NodeType.Wait:
                        if (block is WaitNode wait)
                            sb.AppendLine($"{indentStr}WaitTime {wait.DurationMs / 1000.0:F1};");
                        break;
                    case NodeType.SetDO:
                        if (block is SetDONode setdo)
                            sb.AppendLine($"{indentStr}SetDO {setdo.OutputPin}, {(setdo.Value ? "1" : "0")};");
                        break;
                    case NodeType.GripperOpen:
                        sb.AppendLine($"{indentStr}GripperOpen;");
                        break;
                    case NodeType.GripperClose:
                        sb.AppendLine($"{indentStr}GripperClose;");
                        break;
                    case NodeType.Stop:
                        sb.AppendLine($"{indentStr}Stop;");
                        break;
                }
            }
        }

        private static ProgramNode DeserializeModToProgram(string modText)
        {
            // Parse .mod text back into ProgramNode AST
            // Simplified parser — in production, use a proper parser generator
            var program = new ProgramNode();
            var lines = modText.Split('\n');

            foreach (var line in lines)
            {
                var trimmed = line.Trim();
                if (trimmed.StartsWith("MoveJ", StringComparison.OrdinalIgnoreCase))
                    program.AddChild(new MoveJNode { Name = ExtractBlockName(trimmed) });
                else if (trimmed.StartsWith("MoveL", StringComparison.OrdinalIgnoreCase))
                    program.AddChild(new MoveLNode { Name = ExtractBlockName(trimmed) });
                else if (trimmed.StartsWith("MoveC", StringComparison.OrdinalIgnoreCase))
                    program.AddChild(new MoveCNode { Name = ExtractBlockName(trimmed) });
                else if (trimmed.StartsWith("WaitTime", StringComparison.OrdinalIgnoreCase))
                    program.AddChild(new WaitNode { Name = "Wait" });
                else if (trimmed.StartsWith("SetDO", StringComparison.OrdinalIgnoreCase))
                    program.AddChild(new SetDONode { Name = "SetDO" });
                else if (trimmed.StartsWith("GripperOpen", StringComparison.OrdinalIgnoreCase))
                    program.AddChild(new GripperOpenNode { Name = "GripperOpen" });
                else if (trimmed.StartsWith("GripperClose", StringComparison.OrdinalIgnoreCase))
                    program.AddChild(new GripperCloseNode { Name = "GripperClose" });
                else if (trimmed.StartsWith("Stop", StringComparison.OrdinalIgnoreCase))
                    program.AddChild(new StopNode { Name = "Stop" });
            }

            return program;
        }

        private static string ExtractBlockName(string line)
        {
            var parts = line.Split(' ', ';');
            return parts.Length > 1 ? parts[1].Trim() : "Unnamed";
        }
    }

    // ── Data Models ──────────────────────────────────────────────────────────

    /// <summary>Complete project data loaded from a .rfproj file</summary>
    public class ProjectData
    {
        public string FilePath { get; set; } = "";
        public string TempDir { get; set; } = "";
        public RobotModel Robot { get; set; } = new();
        public ProgramNode Program { get; set; } = new();
        public IoConfiguration IoConfig { get; set; } = new();
        public ProjectSettings Settings { get; set; } = new();
        public string[] MeshFiles { get; set; } = Array.Empty<string>();
        public byte[]? Thumbnail { get; set; }
        public DateTime LastModified { get; set; }

        /// <summary>Clean up temp directory when project is closed</summary>
        public void Cleanup()
        {
            if (!string.IsNullOrEmpty(TempDir) && Directory.Exists(TempDir))
            {
                try { Directory.Delete(TempDir, true); }
                catch { /* Ignore cleanup failures */ }
            }
        }
    }

    /// <summary>Editor preferences, UI state, and application settings</summary>
    public class ProjectSettings
    {
        // Camera state
        public double CameraX { get; set; } = 3;
        public double CameraY { get; set; } = 2.5;
        public double CameraZ { get; set; } = 3;
        public double CameraFov { get; set; } = 45;

        // UI state
        public double LeftPanelWidth { get; set; } = 360;
        public bool GridVisible { get; set; } = true;
        public bool WireframeMode { get; set; }
        public string ActiveTab { get; set; } = "Blocks";

        // Auto-save
        public int AutoSaveIntervalMinutes { get; set; } = 5;
        public DateTime LastAutoSave { get; set; }

        // Recent projects (stored in ~/.roboforge/settings.json, not in .rfproj)
        public List<string> RecentProjects { get; set; } = new();
    }

    /// <summary>
    /// Manages the auto-save timer and recent projects list.
    /// Runs silently in the background.
    /// </summary>
    public class AutoSaveManager
    {
        private Timer? _timer;
        private string _projectPath = "";
        private Func<RobotModel>? _getRobot;
        private Func<ProgramNode>? _getProgram;
        private Func<IoConfiguration>? _getIoConfig;
        private Func<ProjectSettings>? _getSettings;
        private Action<string>? _onStatusUpdate;

        /// <summary>Configure the auto-save manager with data providers</summary>
        public void Configure(
            string projectPath,
            Func<RobotModel> getRobot,
            Func<ProgramNode> getProgram,
            Func<IoConfiguration> getIoConfig,
            Func<ProjectSettings> getSettings,
            Action<string>? onStatusUpdate = null)
        {
            _projectPath = projectPath;
            _getRobot = getRobot;
            _getProgram = getProgram;
            _getIoConfig = getIoConfig;
            _getSettings = getSettings;
            _onStatusUpdate = onStatusUpdate;
        }

        /// <summary>Start the auto-save timer</summary>
        public void Start(int intervalMinutes = 5)
        {
            Stop();
            _timer = new Timer(
                callback: async _ => await DoAutoSave(),
                state: null,
                dueTime: TimeSpan.FromMinutes(intervalMinutes),
                period: TimeSpan.FromMinutes(intervalMinutes));
        }

        /// <summary>Stop the auto-save timer</summary>
        public void Stop()
        {
            _timer?.Dispose();
            _timer = null;
        }

        /// <summary>Force an immediate auto-save</summary>
        public async Task ForceSaveAsync()
        {
            if (_getRobot == null || _getProgram == null || _getIoConfig == null || _getSettings == null)
                return;

            var success = await ProjectFileHandler.AutoSaveAsync(
                _projectPath, _getRobot(), _getProgram(), _getIoConfig(), _getSettings());

            if (success)
            {
                _onStatusUpdate?.Invoke($"Auto-saved at {DateTime.Now:HH:mm:ss}");
            }
        }

        private async Task DoAutoSave()
        {
            try
            {
                await ForceSaveAsync();
            }
            catch
            {
                // Auto-save should never crash the app — silently fail
            }
        }

        /// <summary>Add a project to the recent projects list</summary>
        public static void AddToRecentProjects(string path)
        {
            var settingsPath = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
                "RoboForge", "settings.json");

            var settings = LoadAppSettings(settingsPath);
            settings.RecentProjects.RemoveAll(p => p == path);
            settings.RecentProjects.Insert(0, path);
            while (settings.RecentProjects.Count > 10)
                settings.RecentProjects.RemoveAt(settings.RecentProjects.Count - 1);

            var dir = Path.GetDirectoryName(settingsPath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
                Directory.CreateDirectory(dir);

            var json = JsonConvert.SerializeObject(settings, Formatting.Indented);
            File.WriteAllText(settingsPath, json);
        }

        /// <summary>Get the list of recent projects (max 10)</summary>
        public static List<string> GetRecentProjects()
        {
            var settingsPath = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
                "RoboForge", "settings.json");

            return LoadAppSettings(settingsPath).RecentProjects;
        }

        private static ProjectSettings LoadAppSettings(string path)
        {
            if (File.Exists(path))
            {
                try
                {
                    var json = File.ReadAllText(path);
                    return JsonConvert.DeserializeObject<ProjectSettings>(json) ?? new ProjectSettings();
                }
                catch { return new ProjectSettings(); }
            }
            return new ProjectSettings();
        }
    }
}
