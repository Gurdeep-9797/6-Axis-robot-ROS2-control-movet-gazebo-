#!/usr/bin/env python3
import os
import sys
import argparse
import xml.etree.ElementTree as ET
import shutil

def validate_urdf(urdf_path):
    print(f"Validating URDF: {urdf_path}")
    if not os.path.exists(urdf_path):
        print("Error: URDF file not found.")
        return False
    
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        if root.tag != 'robot':
            print("Error: Root tag is not 'robot'")
            return False
            
        robot_name = root.get('name')
        print(f"Robot Name: {robot_name}")
        
        # Check for mesh paths
        for mesh in root.findall(".//mesh"):
            filename = mesh.get('filename')
            if filename:
                print(f"Checking mesh: {filename}")
                # Ensure paths are package:// relative or relative
                if not filename.startswith("package://") and not filename.startswith("file://"):
                   print(f"Warning: Mesh path {filename} is not a package path.")
                   
        print("Basic Schema Validation: PASS")
        return True
    except ET.ParseError as e:
        print(f"XML Parse Error: {e}")
        return False

def normalize_paths(urdf_path, package_name):
    print(f"Normalizing paths for package: {package_name}")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Update mesh paths to use package://package_name/meshes/
    for mesh in root.findall(".//mesh"):
        filename = mesh.get('filename')
        if filename:
            basename = os.path.basename(filename)
            new_path = f"package://{package_name}/meshes/{basename}"
            mesh.set('filename', new_path)
            print(f"Updated {filename} -> {new_path}")
            
    tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
    print("Paths normalized.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="URDF Post-Processing and Validation")
    parser.add_argument("urdf_path", help="Path to the URDF file")
    parser.add_argument("--package", help="ROS Package Name for path normalization", default="robot_description")
    
    args = parser.parse_args()
    
    if validate_urdf(args.urdf_path):
        normalize_paths(args.urdf_path, args.package)
        sys.exit(0)
    else:
        sys.exit(1)
