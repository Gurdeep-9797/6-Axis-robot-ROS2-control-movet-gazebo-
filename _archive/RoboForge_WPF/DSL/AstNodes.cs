using System.Collections.Generic;

namespace RoboForge_WPF.DSL
{
    public abstract class AstNode
    {
    }

    public class ProgramNode : AstNode
    {
        public List<AstNode> Statements { get; } = new List<AstNode>();
    }

    public class MoveJNode : AstNode
    {
        public string TargetName { get; }
        public double Velocity { get; }
        public int Zone { get; }

        public MoveJNode(string targetName, double velocity, int zone)
        {
            TargetName = targetName;
            Velocity = velocity;
            Zone = zone;
        }
    }

    public class MoveLNode : AstNode
    {
        public string TargetName { get; }
        public double Velocity { get; }
        public int Zone { get; }

        public MoveLNode(string targetName, double velocity, int zone)
        {
            TargetName = targetName;
            Velocity = velocity;
            Zone = zone;
        }
    }

    public class WaitNode : AstNode
    {
        public double DurationMs { get; }

        public WaitNode(double durationMs)
        {
            DurationMs = durationMs;
        }
    }

    public class SetIONode : AstNode
    {
        public int Pin { get; }
        public bool State { get; }

        public SetIONode(int pin, bool state)
        {
            Pin = pin;
            State = state;
        }
    }

    public class WhileNode : AstNode
    {
        public ExpressionNode Condition { get; }
        public List<AstNode> Body { get; } = new List<AstNode>();

        public WhileNode(ExpressionNode condition)
        {
            Condition = condition;
        }
    }

    public class IfNode : AstNode
    {
        public ExpressionNode Condition { get; }
        public List<AstNode> TrueBranch { get; } = new List<AstNode>();
        public List<AstNode> FalseBranch { get; } = new List<AstNode>();

        public IfNode(ExpressionNode condition)
        {
            Condition = condition;
        }
    }

    public abstract class ExpressionNode : AstNode
    {
    }

    public class BinaryOpNode : ExpressionNode
    {
        public ExpressionNode Left { get; }
        public string Operator { get; }
        public ExpressionNode Right { get; }

        public BinaryOpNode(ExpressionNode left, string op, ExpressionNode right)
        {
            Left = left;
            Operator = op;
            Right = right;
        }
    }

    public class LiteralNode : ExpressionNode
    {
        public string Value { get; }
        public string LiteralType { get; } // "Number", "Boolean", "Identifier"

        public LiteralNode(string value, string type)
        {
            Value = value;
            LiteralType = type;
        }
    }
}
