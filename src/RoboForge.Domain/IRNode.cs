using System;
using System.Collections.Generic;

namespace RoboForge.Domain
{
    public abstract record IRNode
    {
        public string Id { get; init; } = Guid.NewGuid().ToString();
        public string Label { get; init; } = string.Empty;
    }

    public record MoveJNode(string Id, string Label, Pose Target, double Speed, ZoneType Zone, string ToolId) : IRNode;
    public record MoveLNode(string Id, string Label, Pose Target, double Speed, double Accel, ZoneType Zone) : IRNode;
    public record MoveCNode(string Id, string Label, Pose Via, Pose Target, double Speed) : IRNode;
    public record WaitNode(string Id, string Label, double DurationSeconds) : IRNode;
    public record SetDONode(string Id, string Label, int Channel, bool Value) : IRNode;
    public record GetDINode(string Id, string Label, int Channel) : IRNode;
    public record IfNode(string Id, string Label, string Condition, List<IRNode> ThenBranch, List<IRNode> ElseBranch) : IRNode;
    public record WhileNode(string Id, string Label, string Condition, List<IRNode> Body) : IRNode;
    public record ProcNode(string Id, string Label, List<IRNode> Body) : IRNode;
    public record ProgramNode(string Id, string Label, List<IRNode> Procedures) : IRNode;
}
