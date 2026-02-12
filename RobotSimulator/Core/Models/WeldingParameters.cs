using System;

namespace RobotSimulator.Core.Models
{
    /// <summary>
    /// Welding parameters based on industrial arc welding standards.
    /// Compatible with FANUC ArcTool and Lincoln PowerWave systems.
    /// </summary>
    public class WeldingParameters
    {
        /// <summary>Wire feed rate in meters per minute (1-25 m/min typical)</summary>
        public double WireFeedRate { get; set; } = 10.0;
        
        /// <summary>Arc voltage in volts (15-40V typical)</summary>
        public double ArcVoltage { get; set; } = 28.0;
        
        /// <summary>Arc current in amperes (50-500A typical)</summary>
        public double ArcCurrent { get; set; } = 250.0;
        
        /// <summary>Travel speed in mm/s (1-100 mm/s)</summary>
        public double TravelSpeed { get; set; } = 15.0;
        
        /// <summary>Weave oscillation pattern type</summary>
        public WeavePattern WeaveType { get; set; } = WeavePattern.None;
        
        /// <summary>Weave width in mm (0-15mm typical)</summary>
        public double WeaveWidth { get; set; } = 4.0;
        
        /// <summary>Weave frequency in Hz (0.5-5 Hz typical)</summary>
        public double WeaveFrequency { get; set; } = 2.0;
        
        /// <summary>Left-side dwell time in ms (0-500ms)</summary>
        public double DwellTimeLeft { get; set; } = 50.0;
        
        /// <summary>Right-side dwell time in ms (0-500ms)</summary>
        public double DwellTimeRight { get; set; } = 50.0;
        
        /// <summary>Weave azimuth angle in degrees (rotation of weave pattern)</summary>
        public double WeaveAzimuth { get; set; } = 0.0;
        
        /// <summary>Gas pre-flow time in seconds</summary>
        public double GasPreFlow { get; set; } = 0.5;
        
        /// <summary>Gas post-flow time in seconds</summary>
        public double GasPostFlow { get; set; } = 1.0;
        
        /// <summary>Arc start retract distance in mm</summary>
        public double ArcStartRetract { get; set; } = 3.0;
        
        /// <summary>Crater fill enabled for end of weld</summary>
        public bool CraterFillEnabled { get; set; } = true;
        
        /// <summary>Crater fill time in seconds</summary>
        public double CraterFillTime { get; set; } = 0.5;
        
        /// <summary>Clone current parameters</summary>
        public WeldingParameters Clone()
        {
            return (WeldingParameters)MemberwiseClone();
        }
        
        /// <summary>Get estimated heat input (kJ/mm)</summary>
        public double GetHeatInput()
        {
            if (TravelSpeed <= 0) return 0;
            // Heat Input = (Voltage × Current × 60) / (Travel Speed × 1000)
            return (ArcVoltage * ArcCurrent * 60.0) / (TravelSpeed * 1000.0);
        }
    }

    /// <summary>Weave oscillation patterns for welding</summary>
    public enum WeavePattern
    {
        /// <summary>No weaving - straight weld</summary>
        None,
        
        /// <summary>Sinusoidal side-to-side motion</summary>
        Sine,
        
        /// <summary>Triangular zigzag pattern</summary>
        Zigzag,
        
        /// <summary>Circular/spiral pattern</summary>
        Circular,
        
        /// <summary>Figure-8 pattern for wide joints</summary>
        Figure8,
        
        /// <summary>L-shaped stitch pattern</summary>
        LPattern,
        
        /// <summary>Crescent/moon pattern</summary>
        Crescent
    }
}
