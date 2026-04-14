// ── Compiler: AST → Instruction List ──────────────────────────────────────
using System;
using System.Collections.Generic;
using RoboForge.Wpf.AST;

namespace RoboForge.Wpf.Core
{
    /// <summary>Compiled instruction for the execution engine</summary>
    public class Instruction
    {
        public string InstructionId { get; set; } = Guid.NewGuid().ToString();
        public string SourceNodeId { get; set; } = ""; // Maps back to AST block
        public NodeType InstructionType { get; set; }
        public Dictionary<string, object> Parameters { get; set; } = new();
        public double EstimatedDuration { get; set; } // seconds
    }

    /// <summary>
    /// Compiles the ProgramAST into a flat InstructionList for execution.
    /// Handles control flow by inserting jump instructions for loops/conditionals.
    /// </summary>
    public static class Compiler
    {
        /// <summary>Compile the entire program AST into a flat instruction list</summary>
        public static List<Instruction> Compile(ProgramNode program)
        {
            var instructions = new List<Instruction>();
            CompileNode(program, instructions);
            return instructions;
        }

        private static void CompileNode(AstNode node, List<Instruction> instructions)
        {
            switch (node.NodeType)
            {
                case NodeType.Program:
                case NodeType.Sequence:
                    foreach (var child in node.Children)
                        CompileNode(child, instructions);
                    break;

                case NodeType.MoveJ:
                case NodeType.MoveL:
                case NodeType.MoveC:
                case NodeType.MoveAbsJ:
                case NodeType.SearchL:
                case NodeType.SetDO:
                case NodeType.PulseDO:
                case NodeType.WaitDI:
                case NodeType.Wait:
                case NodeType.GripperOpen:
                case NodeType.GripperClose:
                case NodeType.Stop:
                case NodeType.Break:
                    instructions.Add(new Instruction
                    {
                        SourceNodeId = node.NodeId,
                        InstructionType = node.NodeType,
                        Parameters = new Dictionary<string, object>(node.Properties),
                        EstimatedDuration = EstimateDuration(node),
                    });
                    break;

                case NodeType.While:
                    {
                        var checkIdx = instructions.Count;
                        // Condition check (will jump to end if false)
                        instructions.Add(new Instruction
                        {
                            SourceNodeId = node.NodeId,
                            InstructionType = NodeType.While,
                            Parameters = new Dictionary<string, object> { ["condition"] = node.Properties.GetValueOrDefault("condition", ""), ["jumpIndex"] = 0 },
                        });
                        // Compile body
                        foreach (var child in node.Children)
                            CompileNode(child, instructions);
                        // Jump back to condition check (use LoopEnd, not Break)
                        instructions.Add(new Instruction
                        {
                            SourceNodeId = node.NodeId + "_loopend",
                            InstructionType = NodeType.LoopEnd,
                            Parameters = new Dictionary<string, object> { ["jumpTo"] = checkIdx },
                        });
                        // Update condition check with jump target (end of loop)
                        instructions[checkIdx].Parameters["jumpIndex"] = instructions.Count;
                        break;
                    }

                case NodeType.If:
                    {
                        var checkIdx = instructions.Count;
                        instructions.Add(new Instruction
                        {
                            SourceNodeId = node.NodeId,
                            InstructionType = NodeType.If,
                            Parameters = new Dictionary<string, object> { ["condition"] = node.Properties.GetValueOrDefault("condition", ""), ["elseJump"] = 0 },
                        });
                        // If branch
                        if (node is IfNode ifNode)
                        {
                            foreach (var child in ifNode.IfBranch.Children)
                                CompileNode(child, instructions);
                        }
                        // Jump over else branch
                        var elseJumpIdx = instructions.Count;
                        instructions.Add(new Instruction
                        {
                            SourceNodeId = node.NodeId + "_endif",
                            InstructionType = NodeType.LoopEnd, // Reuse for jump (not a loop)
                            Parameters = new Dictionary<string, object> { ["jumpTo"] = 0 },
                        });
                        // Update if with else jump target
                        instructions[checkIdx].Parameters["elseJump"] = elseJumpIdx + 1;
                        // Else branch
                        if (node is IfNode ifNode2)
                        {
                            foreach (var child in ifNode2.ElseBranch.Children)
                                CompileNode(child, instructions);
                        }
                        // Update endif jump target
                        instructions[elseJumpIdx].Parameters["jumpTo"] = instructions.Count;
                        break;
                    }

                default:
                    // Unknown node type, skip
                    break;
            }
        }

        private static double EstimateDuration(AstNode node)
        {
            // Simple estimation based on block type
            return node.NodeType switch
            {
                NodeType.MoveJ => 2.0,
                NodeType.MoveL => 2.0,
                NodeType.Wait => node.Properties.TryGetValue("durationMs", out var d) ? (double)d / 1000.0 : 1.0,
                NodeType.SetDO => 0.01,
                NodeType.PulseDO => node.Properties.TryGetValue("durationMs", out var pd) ? (double)pd / 1000.0 : 0.1,
                NodeType.GripperOpen => 1.0,
                NodeType.GripperClose => 1.0,
                _ => 0.1,
            };
        }
    }
}
