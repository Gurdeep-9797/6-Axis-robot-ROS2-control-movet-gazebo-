using System.Collections.Generic;
using System.Text;
using TeachPendant_WPF.Models;

namespace RoboForge_WPF.DSL
{
    public class ScriptGenerator
    {
        public static string Generate(IEnumerable<RobotInstruction> instructions)
        {
            var sb = new StringBuilder();
            sb.AppendLine("// Auto-generated script sequence");
            sb.AppendLine();

            foreach (var inst in instructions)
            {
                if (inst is PtpInstruction ptp)
                {
                    sb.AppendLine($"movej(\"{ptp.PointId}\", {ptp.Speed}, 0);");
                }
                else if (inst is LinInstruction lin)
                {
                    sb.AppendLine($"movel(\"{lin.PointId}\", {lin.Speed}, {lin.Blending});");
                }
                else if (inst is WaitInstruction w)
                {
                    sb.AppendLine($"wait({w.DelayMs});");
                }
                else if (inst is SetDOInstruction sio)
                {
                    sb.AppendLine($"set_io({sio.Port}, {(sio.Value > 0 ? "true" : "false")});");
                }
                else
                {
                    sb.AppendLine($"// Unsupported instruction: {inst.GetType().Name}");
                }
            }
            return sb.ToString();
        }
    }
}
