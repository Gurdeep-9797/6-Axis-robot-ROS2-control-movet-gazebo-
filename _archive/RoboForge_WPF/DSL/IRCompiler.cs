using System;
using System.Collections.Generic;
using TeachPendant_WPF.Models;

namespace RoboForge_WPF.DSL
{
    public class IRCompiler
    {
        public List<RobotInstruction> Compile(ProgramNode astRoot)
        {
            var instructions = new List<RobotInstruction>();
            foreach (var stmt in astRoot.Statements)
            {
                var inst = CompileStatement(stmt);
                if (inst != null) instructions.Add(inst);
            }
            return instructions;
        }

        private RobotInstruction? CompileStatement(AstNode node)
        {
            if (node is MoveJNode mj)
            {
                // MoveJNode maps directly to PtpInstruction
                return new PtpInstruction(mj.TargetName, mj.Velocity);
            }
            if (node is MoveLNode ml)
            {
                return new LinInstruction(ml.TargetName, ml.Velocity, ml.Zone);
            }
            if (node is WaitNode w)
            {
                return new WaitInstruction((int)w.DurationMs);
            }
            if (node is SetIONode sio)
            {
                return new SetDOInstruction(sio.Pin, sio.State ? 1 : 0); 
            }

            return null;
        }
    }
}
