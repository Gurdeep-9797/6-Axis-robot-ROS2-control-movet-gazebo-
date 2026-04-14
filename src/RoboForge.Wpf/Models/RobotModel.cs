// ── Core Models: Robot Structure ──────────────────────────────────────────
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Windows.Media.Media3D;
using CommunityToolkit.Mvvm.ComponentModel;

namespace RoboForge.Wpf.Models
{
    /// <summary>Joint types supported by the robot</summary>
    public enum JointType { Revolute, Prismatic, Fixed, Continuous, Planar, Floating }

    /// <summary>Motor types for joint actuation</summary>
    public enum MotorType { BLDC, Stepper, ServoDC, ServoBrushless }

    /// <summary>Encoder types for joint feedback</summary>
    public enum EncoderType { HallEffect, Quadrature, Absolute, Potentiometer }

    /// <summary>Represents a physical joint between two robot links</summary>
    public partial class JointModel : ObservableObject
    {
        [ObservableProperty] private string _name = "";
        [ObservableProperty] private JointType _jointType = JointType.Revolute;
        [ObservableProperty] private Vector3D _axis = new(0, 0, 1);
        [ObservableProperty] private double _homePosition;
        [ObservableProperty] private double _lowerLimit = -180;
        [ObservableProperty] private double _upperLimit = 180;
        [ObservableProperty] private double _maxVelocity = 180;
        [ObservableProperty] private double _maxAcceleration = 360;
        [ObservableProperty] private double _maxTorque = 100;
        [ObservableProperty] private double _gearRatio = 1;
        [ObservableProperty] private MotorType _motorType = MotorType.BLDC;
        [ObservableProperty] private EncoderType _encoderType = EncoderType.Absolute;
        [ObservableProperty] private double _pidKp = 1.0;
        [ObservableProperty] private double _pidKi = 0.01;
        [ObservableProperty] private double _pidKd = 0.1;
        [ObservableProperty] private int _motorPin = -1;
        [ObservableProperty] private int _encoderPinA = -1;
        [ObservableProperty] private int _encoderPinB = -1;
        [ObservableProperty] private int _hallSensorPin = -1;
        // Runtime state
        [ObservableProperty] private double _currentAngle;
        [ObservableProperty] private bool _isHomed;
    }

    /// <summary>Represents a rigid body segment of the robot</summary>
    public partial class LinkModel : ObservableObject
    {
        [ObservableProperty] private string _name = "";
        [ObservableProperty] private double _mass = 1.0;
        [ObservableProperty] private Vector3D _comOffset = new();
        [ObservableProperty] private Vector3D _inertia = new(0.01, 0.01, 0.01);
        [ObservableProperty] private string _meshFile = "";
        [ObservableProperty] private string _color = "#E8E8E8";
        [ObservableProperty] private double _opacity = 100;
        [ObservableProperty] private bool _wireframe;
        [ObservableProperty] private string _collisionType = "Box";
        [ObservableProperty] private Vector3D _collisionDimensions = new(0.1, 0.1, 0.1);
        [ObservableProperty] private bool _isVisible = true;
        [ObservableProperty] private bool _isHealthy = true;
        [ObservableProperty] private bool _hasWarning;
        [ObservableProperty] private bool _hasError;
        public ObservableCollection<JointModel> ChildJoints { get; } = new();
    }

    /// <summary>Sensor attached to a robot link</summary>
    public partial class SensorModel : ObservableObject
    {
        [ObservableProperty] private string _name = "";
        [ObservableProperty] private string _sensorType = "Hall Effect";
        [ObservableProperty] private int _pin = -1;
        [ObservableProperty] private bool _activeHigh = true;
        [ObservableProperty] private int _debounceMs = 10;
        [ObservableProperty] private double _calibrationOffset;
        [ObservableProperty] private double _currentValue;
        [ObservableProperty] private bool _isHealthy = true;
    }

    /// <summary>End-effector tool configuration</summary>
    public partial class EndEffectorModel : ObservableObject
    {
        [ObservableProperty] private string _name = "Gripper";
        [ObservableProperty] private Vector3D _tcpOffset = new();
        [ObservableProperty] private Vector3D _tcpRotation = new();
        [ObservableProperty] private string _toolType = "Gripper";
        [ObservableProperty] private double _openWidth = 80;
        [ObservableProperty] private double _closedWidth = 0;
        [ObservableProperty] private double _maxForce = 50;
        [ObservableProperty] private double _currentWidth;
        [ObservableProperty] private bool _isGripping;
    }

    /// <summary>Coordinate frame definition (world, base, tool, work object)</summary>
    public partial class CoordinateFrameModel : ObservableObject
    {
        [ObservableProperty] private string _name = "";
        [ObservableProperty] private string _parentFrame = "world";
        [ObservableProperty] private Vector3D _position = new();
        [ObservableProperty] private Vector3D _rotation = new(); // RPY in degrees
        [ObservableProperty] private string _color = "#00D4FF";
    }

    /// <summary>Root robot model containing the entire physical structure</summary>
    public partial class RobotModel : ObservableObject
    {
        [ObservableProperty] private string _name = "My Robot";
        [ObservableProperty] private string _manufacturer = "";
        [ObservableProperty] private string _model = "";
        [ObservableProperty] private string _frameConvention = "DH";
        [ObservableProperty] private int _degreesOfFreedom;

        public ObservableCollection<LinkModel> Links { get; } = new();
        public ObservableCollection<SensorModel> Sensors { get; } = new();
        public EndEffectorModel EndEffector { get; } = new();
        public ObservableCollection<CoordinateFrameModel> Frames { get; } = new();

        public int TotalJoints => CountJoints(Links);
        private int CountJoints(ObservableCollection<LinkModel> links)
        {
            int count = 0;
            foreach (var link in links) count += link.ChildJoints.Count + CountJointsFromChildren(link);
            return count;
        }
        private int CountJointsFromChildren(LinkModel link) => 0; // Simplified for now
    }
}
