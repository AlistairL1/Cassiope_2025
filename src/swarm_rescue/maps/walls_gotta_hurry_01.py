"""
This file defines the walls for the map_gotta_hurry_01.py map.
It creates a narrow corridor with walls on both sides.
"""
import sys
from pathlib import Path

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from spg_overlay.entities.normal_wall import NormalWall


def add_boxes(_):
    """
    Function to add boxes to the playground.
    Not used in this map.
    """
    pass


def add_walls(playground):
    """
    Function to add walls to the playground.
    Creates a narrow corridor with top and bottom walls.
    """
    corridor_width = 200  # Width of the corridor (less than lidar range of 300)
    corridor_length = 1500  # Length of the corridor
    wall_thickness = 20  # Thickness of the walls

    # Top wall of corridor (long wall)
    wall_top = NormalWall(pos_start=(-corridor_length/2, corridor_width/2),
                          pos_end=(corridor_length/2, corridor_width/2),
                          width=wall_thickness)
    playground.add(wall_top, wall_top.wall_coordinates)

    # Bottom wall of corridor (long wall)
    wall_bottom = NormalWall(pos_start=(-corridor_length/2, -corridor_width/2),
                             pos_end=(corridor_length/2, -corridor_width/2),
                             width=wall_thickness)
    playground.add(wall_bottom, wall_bottom.wall_coordinates)

    # Left end wall (optional - closes the left end of the corridor)
    wall_left = NormalWall(pos_start=(-corridor_length/2, corridor_width/2),
                           pos_end=(-corridor_length/2, -corridor_width/2),
                           width=wall_thickness)
    playground.add(wall_left, wall_left.wall_coordinates)

    # Right end wall (optional - closes the right end of the corridor)
    wall_right = NormalWall(pos_start=(corridor_length/2, corridor_width/2),
                            pos_end=(corridor_length/2, -corridor_width/2),
                            width=wall_thickness)
    playground.add(wall_right, wall_right.wall_coordinates) 