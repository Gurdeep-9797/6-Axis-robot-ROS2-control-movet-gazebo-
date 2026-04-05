using System;
using System.Collections.Generic;
using System.Linq;
using RoboForge.Domain;
using RoboForge.ROS2Bridge.Messages;

// Mock dependencies to allow structure to compile
namespace RoboForge.ROS2Bridge.Messages
{
    public class MotionSequenceItem 
    { 
        public MotionPlanRequest Req { get; set; }
        public double BlendRadius { get; set; }
    }
    public class MotionPlanRequest 
    {
        public string GroupName { get; set; }
        public string PlannerConfig { get; set; }
        public double MaxVelocityScalingFactor { get; set; }
        public object[] GoalConstraints { get; set; }
        public object[] PathConstraints { get; set; }
    }
    public class MotionSequenceRequest 
    {
        public List<MotionSequenceItem> Items { get; set; }
    }
}

namespace RoboForge.Application
{
    public class IRToRos2GoalTranslator
    {
        public MoveGroupSequenceActionGoal Translate(IReadOnlyList<IRNode> nodes, double speedOverride)
        {
            var items = new List<MotionSequenceItem>();
            foreach (var node in nodes)
            {
                var item = node switch {
                    MoveJNode j => BuildMoveJItem(j, speedOverride),
                    MoveLNode l => BuildMoveLItem(l, speedOverride),
                    MoveCNode c => BuildMoveCItem(c, speedOverride),
                    WaitNode  w => null,  // waits are handled by execution state machine
                    SetDONode d => null,  // IO handled separately
                    GetDINode g => null,
                    _ => throw new NotSupportedException($"Node type {node.GetType().Name} not supported in translator.")
                };
                if (item != null) items.Add(item);
            }
            
            // In a real RCLLibrary this would map directly to the message
            return new MoveGroupSequenceActionGoal { 
                /* Request = new MotionSequenceRequest { Items = items } */ 
            };
        }

        private MotionSequenceItem BuildMoveJItem(MoveJNode node, double speedOverride) => new MotionSequenceItem {
            Req = new MotionPlanRequest {
                GroupName = "manipulator",
                PlannerConfig = "PTP",   // PILZ point-to-point = joint space
                MaxVelocityScalingFactor = (node.Speed / 5000.0) * speedOverride,
                GoalConstraints = new[] { BuildJointGoal(node.Target) }
            },
            BlendRadius = ZoneToRadius(node.Zone)
        };

        private MotionSequenceItem BuildMoveLItem(MoveLNode node, double speedOverride) => new MotionSequenceItem {
            Req = new MotionPlanRequest {
                GroupName = "manipulator",
                PlannerConfig = "LIN",   // PILZ straight line
                MaxVelocityScalingFactor = (node.Speed / 5000.0) * speedOverride,
                GoalConstraints = new[] { BuildCartesianGoal(node.Target) },
                PathConstraints = new object[0] // Straight line constraint
            },
            BlendRadius = ZoneToRadius(node.Zone)
        };

        private MotionSequenceItem BuildMoveCItem(MoveCNode node, double speedOverride) => new MotionSequenceItem {
            Req = new MotionPlanRequest {
                GroupName = "manipulator",
                PlannerConfig = "CIRC",   // PILZ circular
                MaxVelocityScalingFactor = (node.Speed / 5000.0) * speedOverride,
                GoalConstraints = new[] { BuildCartesianGoal(node.Target), BuildCartesianGoal(node.Via) } // Simplified
            },
            BlendRadius = 0 // typically fine for CIRC
        };

        private double ZoneToRadius(ZoneType z) => z switch {
            ZoneType.Fine => 0.0,
            ZoneType.Z1   => 0.001,
            ZoneType.Z5   => 0.005,
            ZoneType.Z10  => 0.010,
            ZoneType.Z25  => 0.025,
            ZoneType.Z50  => 0.050,
            ZoneType.Z100 => 0.100,
            _ => 0.0
        };

        private object BuildJointGoal(Pose pose) => new object(); // stub
        private object BuildCartesianGoal(Pose pose) => new object(); // stub
    }
}
