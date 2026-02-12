#!/usr/bin/env python3
"""
SolidWorks 2020 → URDF Export Pipeline Stub
============================================
This module provides integration with SolidWorks for URDF export.

CURRENT STATUS: STUBBED
Reason: SolidWorks API requires Windows COM automation which is not
available in the Docker/Linux ROS 2 environment.

When running on Windows with SolidWorks installed:
1. Open SolidWorks 2020 with the robot assembly
2. Install sw_urdf_exporter add-in (v1.6.0)
3. Use Export → URDF from SolidWorks menu
4. Copy output to src/robot_description/urdf/

This script provides:
- API availability detection
- Graceful fallback to existing URDF
- Logging of stub status
"""

import os
import sys
import logging
from datetime import datetime

LOG_DIR = '/ros_ws/logs/sim_validation' if os.path.exists('/ros_ws') else 'logs/sim_validation'
LOG_FILE = os.path.join(LOG_DIR, 'solidworks_integration.log')

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler(LOG_FILE, mode='a') if os.path.exists(os.path.dirname(LOG_FILE)) else logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

def check_solidworks_api():
    """
    Check if SolidWorks API is available.
    
    Returns:
        tuple: (available: bool, reason: str)
    """
    # Check platform
    if sys.platform != 'win32':
        return False, "SolidWorks API requires Windows. Current platform: " + sys.platform
    
    # Try to import COM automation
    try:
        import win32com.client
    except ImportError:
        return False, "pywin32 not installed. Install with: pip install pywin32"
    
    # Try to connect to SolidWorks
    try:
        sw_app = win32com.client.Dispatch("SldWorks.Application")
        version = sw_app.RevisionNumber()
        logger.info(f"SolidWorks detected: Version {version}")
        return True, f"SolidWorks {version} connected"
    except Exception as e:
        return False, f"SolidWorks not running or not installed: {e}"

def export_urdf_from_solidworks(output_dir, assembly_name=None):
    """
    Export URDF from currently open SolidWorks assembly.
    
    Args:
        output_dir: Directory to save URDF files
        assembly_name: Optional name for the robot (defaults to assembly name)
    
    Returns:
        tuple: (success: bool, urdf_path: str or None, message: str)
    """
    available, reason = check_solidworks_api()
    
    if not available:
        logger.warning(f"SolidWorks API not available: {reason}")
        logger.info("Falling back to existing URDF in robot_description package")
        return False, None, reason
    
    # If API is available, attempt export
    try:
        import win32com.client
        sw_app = win32com.client.Dispatch("SldWorks.Application")
        
        # Get active document
        model = sw_app.ActiveDoc
        if model is None:
            return False, None, "No active document in SolidWorks"
        
        # Check if it's an assembly
        doc_type = model.GetType()
        if doc_type != 2:  # swDocASSEMBLY = 2
            return False, None, "Active document is not an assembly"
        
        # Get assembly name
        if assembly_name is None:
            assembly_name = os.path.splitext(os.path.basename(model.GetPathName()))[0]
        
        logger.info(f"Exporting URDF for assembly: {assembly_name}")
        
        # NOTE: Actual URDF export requires sw_urdf_exporter add-in
        # This would be invoked via SolidWorks add-in API if available
        # For now, we stub this functionality
        
        urdf_path = os.path.join(output_dir, f"{assembly_name}.urdf")
        
        logger.warning("sw_urdf_exporter add-in integration not implemented")
        logger.info("Manual export required: Use SolidWorks menu → Tools → Export as URDF")
        
        return False, None, "Export functionality stubbed - use manual export"
        
    except Exception as e:
        logger.error(f"Export failed: {e}")
        return False, None, str(e)

def get_fallback_urdf():
    """
    Get path to fallback URDF.
    
    Returns:
        str: Path to existing URDF file
    """
    urdf_paths = [
        '/ros_ws/src/robot_description/urdf/custom_6axis_test.urdf.xacro',
        'src/robot_description/urdf/custom_6axis_test.urdf.xacro',
    ]
    
    for path in urdf_paths:
        if os.path.exists(path):
            logger.info(f"Using fallback URDF: {path}")
            return path
    
    logger.error("No fallback URDF found!")
    return None

def main():
    """Main entry point for SolidWorks integration."""
    os.makedirs(LOG_DIR, exist_ok=True)
    
    logger.info("=" * 60)
    logger.info("SolidWorks 2020 → URDF Pipeline")
    logger.info("=" * 60)
    
    available, reason = check_solidworks_api()
    
    if available:
        logger.info(f"SolidWorks API Available: {reason}")
        success, urdf_path, message = export_urdf_from_solidworks(
            output_dir='src/robot_description/urdf'
        )
        if success:
            logger.info(f"URDF exported to: {urdf_path}")
        else:
            logger.warning(f"Export failed: {message}")
            fallback = get_fallback_urdf()
            logger.info(f"Using fallback: {fallback}")
    else:
        logger.info(f"SolidWorks API Not Available: {reason}")
        logger.info("This is expected when running in Docker/Linux environment")
        fallback = get_fallback_urdf()
        logger.info(f"Using fallback URDF: {fallback}")
    
    logger.info("=" * 60)
    logger.info("Integration check complete")
    logger.info("=" * 60)
    
    # Return status for caller
    return {
        'solidworks_available': available,
        'reason': reason,
        'fallback_urdf': get_fallback_urdf(),
        'status': 'STUBBED' if not available else 'AVAILABLE'
    }

if __name__ == '__main__':
    result = main()
    print(f"\nResult: {result}")
