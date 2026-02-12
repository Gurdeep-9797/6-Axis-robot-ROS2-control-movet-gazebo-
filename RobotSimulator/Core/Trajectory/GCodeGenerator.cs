using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using RobotSimulator.Core.Models;

namespace RobotSimulator.Core.Trajectory
{
    /// <summary>
    /// Generates G-code and FANUC TP-style code from robot programs.
    /// Compatible with CNC controllers and FANUC Robot G-code feature.
    /// </summary>
    public class GCodeGenerator
    {
        private readonly RobotProgram _program;
        private int _lineNumber = 10;
        private const int LINE_INCREMENT = 10;

        public GCodeGenerator(RobotProgram program)
        {
            _program = program;
        }

        /// <summary>
        /// Generate standard G-code for welding trajectory.
        /// Compatible with CNC/Robot G-code controllers.
        /// </summary>
        public string GenerateGCode()
        {
            var sb = new StringBuilder();
            _lineNumber = 10;

            // Header
            sb.AppendLine($"; FANUC Welding Robot G-Code Program");
            sb.AppendLine($"; Program: {_program.Name}");
            sb.AppendLine($"; Generated: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            sb.AppendLine($"; Points: {_program.Points.Count}");
            sb.AppendLine($"; Est. Cycle Time: {_program.EstimateCycleTime():F1}s");
            sb.AppendLine();
            sb.AppendLine("G21         ; Units: mm");
            sb.AppendLine("G90         ; Absolute positioning");
            sb.AppendLine("G28         ; Home all axes");
            sb.AppendLine();

            bool arcOn = false;
            bool gasOn = false;

            foreach (var point in _program.Points)
            {
                var pos = point.CartesianPosition;
                double x = pos.X * 1000; // Convert to mm
                double y = pos.Y * 1000;
                double z = pos.Z * 1000;
                double rollDeg = point.Roll * 180 / Math.PI;
                double pitchDeg = point.Pitch * 180 / Math.PI;
                double yawDeg = point.Yaw * 180 / Math.PI;

                // Comment
                if (!string.IsNullOrEmpty(point.Comment))
                    sb.AppendLine($"; {point.Comment}");

                // Gas control
                if (point.WeldingEnabled && !gasOn)
                {
                    sb.AppendLine(FormatLine("M08", "Gas ON"));
                    gasOn = true;
                    if (point.WeldParams?.GasPreFlow > 0)
                        sb.AppendLine(FormatLine($"G04 P{point.WeldParams.GasPreFlow:F2}", "Pre-flow delay"));
                }

                // Arc control
                if (point.WeldingEnabled && !arcOn)
                {
                    var weld = point.WeldParams ?? _program.DefaultWeldParams;
                    sb.AppendLine(FormatLine($"M03 S{weld.ArcCurrent:F0}", $"Arc ON ({weld.ArcCurrent:F0}A/{weld.ArcVoltage:F0}V)"));
                    arcOn = true;
                }
                else if (!point.WeldingEnabled && arcOn)
                {
                    sb.AppendLine(FormatLine("M05", "Arc OFF"));
                    arcOn = false;
                    if (_program.DefaultWeldParams.GasPostFlow > 0)
                        sb.AppendLine(FormatLine($"G04 P{_program.DefaultWeldParams.GasPostFlow:F2}", "Post-flow delay"));
                    sb.AppendLine(FormatLine("M09", "Gas OFF"));
                    gasOn = false;
                }

                // Motion command
                string gCode;
                string feedStr = "";

                switch (point.Motion)
                {
                    case MotionType.Joint:
                        gCode = "G00"; // Rapid (joint-like)
                        break;
                    case MotionType.Linear:
                        gCode = "G01";
                        feedStr = $" F{point.Speed * 60:F0}"; // Convert mm/s to mm/min
                        break;
                    case MotionType.Circular:
                        gCode = "G02"; // CW arc (could compute direction)
                        feedStr = $" F{point.Speed * 60:F0}";
                        break;
                    default:
                        gCode = "G01";
                        break;
                }

                // Position with orientation (A, B, C for RPY)
                var cmd = $"{gCode} X{x:F3} Y{y:F3} Z{z:F3} A{rollDeg:F2} B{pitchDeg:F2} C{yawDeg:F2}{feedStr}";
                sb.AppendLine(FormatLine(cmd, $"Point {point.Id}: {point.Name}"));

                // Weave pattern (if welding with weave)
                if (point.WeldingEnabled && point.WeldParams?.WeaveType != WeavePattern.None)
                {
                    var weld = point.WeldParams!;
                    sb.AppendLine($"; Weave: {weld.WeaveType}, Width={weld.WeaveWidth:F1}mm, Freq={weld.WeaveFrequency:F1}Hz");
                }
            }

            // Cleanup
            if (arcOn)
            {
                sb.AppendLine(FormatLine("M05", "Arc OFF"));
                sb.AppendLine(FormatLine($"G04 P{_program.DefaultWeldParams.GasPostFlow:F2}", "Post-flow"));
            }
            if (gasOn)
            {
                sb.AppendLine(FormatLine("M09", "Gas OFF"));
            }

            // Footer
            sb.AppendLine();
            sb.AppendLine(FormatLine("G00 Z50", "Retract Z"));
            sb.AppendLine(FormatLine("G28", "Return home"));
            sb.AppendLine(FormatLine("M30", "End program"));

            return sb.ToString();
        }

        /// <summary>
        /// Generate FANUC TP-style code for teach pendant.
        /// </summary>
        public string GenerateFanucTP()
        {
            var sb = new StringBuilder();
            int lineNum = 1;

            // Header
            sb.AppendLine($"/PROG {_program.Name.ToUpper()}");
            sb.AppendLine("/ATTR");
            sb.AppendLine("OWNER = MNEDITOR;");
            sb.AppendLine("COMMENT = \"Welding Program\";");
            sb.AppendLine($"CREATE = DATE {DateTime.Now:yy-MM-dd} TIME {DateTime.Now:HH:mm:ss};");
            sb.AppendLine($"MODIFIED = DATE {DateTime.Now:yy-MM-dd} TIME {DateTime.Now:HH:mm:ss};");
            sb.AppendLine("/MN");

            bool arcOn = false;

            foreach (var point in _program.Points)
            {
                // Motion instruction
                string motion = point.Motion switch
                {
                    MotionType.Joint => "J",
                    MotionType.Linear => "L",
                    MotionType.Circular => "C",
                    _ => "L"
                };

                string speed = point.Motion == MotionType.Joint
                    ? $"{point.Speed:F0}%"
                    : $"{point.Speed:F0}mm/sec";

                string term = point.Termination == TerminationType.Fine
                    ? "FINE"
                    : $"CNT{point.CntValue}";

                string weldCmd = "";
                if (point.WeldingEnabled && !arcOn)
                {
                    weldCmd = " Arc Start[1]";
                    arcOn = true;
                }
                else if (!point.WeldingEnabled && arcOn)
                {
                    weldCmd = " Arc End[1]";
                    arcOn = false;
                }

                sb.AppendLine($"   {lineNum}:  {motion} P[{point.Id}] {speed} {term}{weldCmd} ;");
                lineNum++;
            }

            // End
            sb.AppendLine($"   {lineNum}:  END ;");
            sb.AppendLine("/POS");

            // Position data
            foreach (var point in _program.Points)
            {
                var pos = point.CartesianPosition;
                var joints = point.GetJointAnglesDegrees();

                sb.AppendLine($"P[{point.Id}]{{");
                sb.AppendLine($"   GP1:");
                sb.AppendLine($"   UF : 0, UT : 1,    CONFIG : 'N U T, 0, 0, 0',");
                sb.AppendLine($"   X = {pos.X * 1000:F3} mm, Y = {pos.Y * 1000:F3} mm, Z = {pos.Z * 1000:F3} mm,");
                sb.AppendLine($"   W = {point.Yaw * 180 / Math.PI:F3} deg, P = {point.Pitch * 180 / Math.PI:F3} deg, R = {point.Roll * 180 / Math.PI:F3} deg");
                sb.AppendLine("};");
            }

            sb.AppendLine("/END");

            return sb.ToString();
        }

        /// <summary>
        /// Export G-code to file
        /// </summary>
        public void ExportGCode(string filePath)
        {
            var code = GenerateGCode();
            File.WriteAllText(filePath, code);
        }

        /// <summary>
        /// Export FANUC TP to file
        /// </summary>
        public void ExportFanucTP(string filePath)
        {
            var code = GenerateFanucTP();
            File.WriteAllText(filePath, code);
        }

        private string FormatLine(string command, string comment)
        {
            var line = $"N{_lineNumber} {command,-40} ; {comment}";
            _lineNumber += LINE_INCREMENT;
            return line;
        }
    }

    /// <summary>
    /// M-Code definitions for welding operations
    /// </summary>
    public static class WeldingMCodes
    {
        public const string ArcOn = "M03";       // Arc/spindle on
        public const string ArcOff = "M05";      // Arc/spindle off
        public const string GasOn = "M08";       // Gas/coolant on
        public const string GasOff = "M09";      // Gas/coolant off
        public const string WireInch = "M10";    // Wire feed inch
        public const string WireRetract = "M11"; // Wire retract
        public const string ProgramEnd = "M30";  // End program
        public const string Dwell = "G04";       // Dwell/delay
    }
}
