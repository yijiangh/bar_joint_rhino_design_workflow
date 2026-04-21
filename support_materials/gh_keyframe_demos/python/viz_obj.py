#!/usr/bin/env python3
"""
Script to load multiple OBJ and URDF files using pybullet_planning with automatic scaling.
If the bounding box exceeds 10 meters, the model is assumed to be in mm units and scaled down by 1000.
"""

import os
import numpy as np
import pybullet_planning as pp
import glob

def load_obj_with_auto_scale(obj_filepath, scale=1.0):
    """
    Load an OBJ file with automatic scaling if the bounding box is too large.
    
    Args:
        obj_filepath: Path to the OBJ file
        scale: Initial scale factor
        
    Returns:
        body_id: PyBullet body ID of the loaded object
        final_scale: The scale factor that was actually applied
    """
    # Check if file exists
    if not os.path.exists(obj_filepath):
        raise FileNotFoundError(f"OBJ file not found: {obj_filepath}")
    
    print(f"Loading OBJ file: {os.path.basename(obj_filepath)}")
    print(f"Initial scale: {scale}")
    
    # Load the object with initial scale
    body_id = pp.create_obj(obj_filepath, scale=scale)
    
    # Get the bounding box
    aabb_min, aabb_max = pp.get_aabb(body_id)
    bbox_size = np.array(aabb_max) - np.array(aabb_min)
    max_dimension = np.max(bbox_size)
    
    print(f"Bounding box size: {bbox_size}")
    print(f"Maximum dimension: {max_dimension:.3f} meters")
    
    # Check if the object is too large (likely in mm units)
    if max_dimension > 10.0:
        print(f"Object is too large ({max_dimension:.3f}m), assuming mm units. Scaling down by 1000...")
        
        # Remove the current object
        pp.remove_body(body_id)
        
        # Reload with scaled down size
        new_scale = scale / 1000.0
        body_id = pp.create_obj(obj_filepath, scale=new_scale)
        
        # Get the new bounding box
        aabb_min, aabb_max = pp.get_aabb(body_id)
        bbox_size = np.array(aabb_max) - np.array(aabb_min)
        max_dimension = np.max(bbox_size)
        
        print(f"After scaling - Bounding box size: {bbox_size}")
        print(f"After scaling - Maximum dimension: {max_dimension:.3f} meters")
        print(f"Applied scale factor: {new_scale}")
        
        return body_id, new_scale
    else:
        print(f"Object size is reasonable, using original scale: {scale}")
        return body_id, scale

def load_urdf_with_auto_scale(urdf_filepath, scale=1.0):
    """
    Load a URDF file with automatic scaling if the bounding box is too large.
    
    Args:
        urdf_filepath: Path to the URDF file
        scale: Initial scale factor
        
    Returns:
        body_id: PyBullet body ID of the loaded object
        final_scale: The scale factor that was actually applied
    """
    # Check if file exists
    if not os.path.exists(urdf_filepath):
        raise FileNotFoundError(f"URDF file not found: {urdf_filepath}")
    
    print(f"Loading URDF file: {os.path.basename(urdf_filepath)}")
    print(f"Initial scale: {scale}")
    
    # Load the URDF with initial scale
    body_id = pp.load_pybullet(urdf_filepath, fixed_base=False, scale=scale)
    
    # Get the bounding box
    aabb_min, aabb_max = pp.get_aabb(body_id)
    bbox_size = np.array(aabb_max) - np.array(aabb_min)
    max_dimension = np.max(bbox_size)
    
    print(f"Bounding box size: {bbox_size}")
    print(f"Maximum dimension: {max_dimension:.3f} meters")
    
    # Check if the object is too large (likely in mm units)
    if max_dimension > 10.0:
        print(f"Object is too large ({max_dimension:.3f}m), assuming mm units. Scaling down by 1000...")
        
        # Remove the current object
        pp.remove_body(body_id)
        
        # Reload with scaled down size
        new_scale = scale / 1000.0
        body_id = pp.load_pybullet(urdf_filepath, fixed_base=False, scale=new_scale)
        
        # Get the new bounding box
        aabb_min, aabb_max = pp.get_aabb(body_id)
        bbox_size = np.array(aabb_max) - np.array(aabb_min)
        max_dimension = np.max(bbox_size)
        
        print(f"After scaling - Bounding box size: {bbox_size}")
        print(f"After scaling - Maximum dimension: {max_dimension:.3f} meters")
        print(f"Applied scale factor: {new_scale}")
        
        return body_id, new_scale
    else:
        print(f"Object size is reasonable, using original scale: {scale}")
        return body_id, scale

def find_files_in_cache(cache_dir):
    """
    Find all URDF and OBJ files in the cache directory.
    
    Args:
        cache_dir: Path to the cache directory
        
    Returns:
        urdf_files: List of URDF file paths
        obj_files: List of OBJ file paths
    """
    urdf_files = glob.glob(os.path.join(cache_dir, "*.urdf"))
    obj_files = glob.glob(os.path.join(cache_dir, "*.obj"))
    
    return urdf_files, obj_files

def load_multiple_files(cache_dir, load_urdfs=True, load_objs=True, urdf_spacing=0.2, obj_spacing=0.2):
    """
    Load multiple URDF and OBJ files from the cache directory.
    
    Args:
        cache_dir: Path to the cache directory
        load_urdfs: Whether to load URDF files
        load_objs: Whether to load OBJ files
        urdf_spacing: Spacing between URDF objects
        obj_spacing: Spacing between OBJ objects
        
    Returns:
        loaded_objects: List of (body_id, filename, file_type, scale) tuples
    """
    loaded_objects = []
    
    # Find all files
    urdf_files, obj_files = find_files_in_cache(cache_dir)
    
    print(f"Found {len(urdf_files)} URDF files and {len(obj_files)} OBJ files")
    
    # Load URDF files
    if load_urdfs and urdf_files:
        print("\n" + "=" * 60)
        print("Loading URDF files:")
        print("=" * 60)
        
        for i, urdf_file in enumerate(urdf_files):
            try:
                print(f"\n--- Loading URDF {i+1}/{len(urdf_files)} ---")
                body_id, scale = load_urdf_with_auto_scale(urdf_file)
                
                # Position the object with spacing
                pos = [i * urdf_spacing, 0, 0]
                pp.set_pose(body_id, pp.Pose(point=pp.Point(*pos)))
                
                # Set different colors for each object
                colors = [pp.RED, pp.GREEN, pp.BLUE, pp.YELLOW, pp.BROWN]
                color = colors[i % len(colors)]
                pp.set_color(body_id, color)
                pp.add_text(os.path.basename(urdf_file), position=np.array(pp.get_pose(body_id)[0]) + np.array([0, 0, 0.1]))
                
                loaded_objects.append((body_id, os.path.basename(urdf_file), "URDF", scale))
                
            except Exception as e:
                print(f"Error loading URDF {urdf_file}: {e}")
    
    # Load OBJ files
    if load_objs and obj_files:
        print("\n" + "=" * 60)
        print("Loading OBJ files:")
        print("=" * 60)
        
        for i, obj_file in enumerate(obj_files):
            try:
                print(f"\n--- Loading OBJ {i+1}/{len(obj_files)} ---")
                body_id, scale = load_obj_with_auto_scale(obj_file)
                
                # Position the object with spacing (offset from URDFs)
                pos = [i * obj_spacing, 0.3, 0]  # Offset in Y direction
                pp.set_pose(body_id, pp.Pose(point=pp.Point(*pos)))
                
                # Set different colors for each object
                colors = [pp.RED, pp.GREEN, pp.BLUE, pp.YELLOW, pp.BROWN]
                color = colors[i % len(colors)]
                pp.set_color(body_id, color)
                pp.add_text(os.path.basename(obj_file), position=np.array(pp.get_pose(body_id)[0]) + np.array([0, 0, 0.1]))
                
                loaded_objects.append((body_id, os.path.basename(obj_file), "OBJ", scale))
                
            except Exception as e:
                print(f"Error loading OBJ {obj_file}: {e}")
    
    return loaded_objects

def main():
    """Main function to load and visualize multiple OBJ and URDF files."""
    
    # Configuration
    LOAD_URDFS = True   # Set to True to load URDF files
    LOAD_OBJS = True    # Set to True to load OBJ files
    
    # Get the cache directory path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cache_dir = os.path.join(script_dir, "..", "..", "250808_cindy_calibration_validation", "tool_urdf_cache")
    
    print("=" * 60)
    print("Multiple File Loader with Auto-Scaling")
    print("=" * 60)
    print(f"Cache directory: {cache_dir}")
    
    # Check if cache directory exists
    if not os.path.exists(cache_dir):
        print(f"Error: Cache directory not found: {cache_dir}")
        return 1
    
    # Initialize PyBullet
    pp.connect(use_gui=True)
    pp.set_camera_pose(camera_point=[0.5, 0.5, 0.5])
    
    try:
        # Load multiple files
        loaded_objects = load_multiple_files(
            cache_dir, 
            load_urdfs=LOAD_URDFS, 
            load_objs=LOAD_OBJS
        )
        
        # Print summary
        print("\n" + "=" * 60)
        print("Loading Summary:")
        print("=" * 60)
        print(f"Successfully loaded {len(loaded_objects)} objects:")
        
        for body_id, filename, file_type, scale in loaded_objects:
            aabb_min, aabb_max = pp.get_aabb(body_id)
            bbox_size = np.array(aabb_max) - np.array(aabb_min)
            print(f"  - {filename} ({file_type}): Body ID {body_id}, Scale {scale:.3f}, BBox {bbox_size}")
        
        print("=" * 60)
        
    except Exception as e:
        print(f"Error: {e}")
        pp.disconnect()
        return 1

    pp.wait_if_gui()
    pp.disconnect()
    
    return 0

if __name__ == "__main__":
    exit(main())
