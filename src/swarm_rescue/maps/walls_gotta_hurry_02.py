"""
This file defines the walls for the map_gotta_hurry_02.py map.
It creates a maze-like structure.
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
    Creates a maze-like structure.
    """
    # Map dimensions
    map_width = 1600
    map_height = 1000
    wall_thickness = 15
    
    # Outer walls
    # Top wall
    wall_top = NormalWall(pos_start=(-map_width/2, map_height/2),
                          pos_end=(map_width/2, map_height/2),
                          width=wall_thickness)
    playground.add(wall_top, wall_top.wall_coordinates)
    
    # Bottom wall
    wall_bottom = NormalWall(pos_start=(-map_width/2, -map_height/2),
                             pos_end=(map_width/2, -map_height/2),
                             width=wall_thickness)
    playground.add(wall_bottom, wall_bottom.wall_coordinates)
    
    # Left wall
    wall_left = NormalWall(pos_start=(-map_width/2, map_height/2),
                           pos_end=(-map_width/2, -map_height/2),
                           width=wall_thickness)
    playground.add(wall_left, wall_left.wall_coordinates)
    
    # Right wall
    wall_right = NormalWall(pos_start=(map_width/2, map_height/2),
                            pos_end=(map_width/2, -map_height/2),
                            width=wall_thickness)
    playground.add(wall_right, wall_right.wall_coordinates)
    
    # Internal maze walls

    wall = NormalWall(pos_start=(-map_width/2, map_height/2-200),
                          pos_end=(0, map_height/2-200),
                          width=wall_thickness)
    playground.add(wall, wall.wall_coordinates)

    wall = NormalWall(pos_start=(600, 128),
                      pos_end=(600, -120),
                      width=wall_thickness)
    playground.add(wall, wall.wall_coordinates)

    wall = NormalWall(pos_start=(240, -map_height/2+190),
                      pos_end=(-240, -map_height/2+190),
                      width=wall_thickness)
    playground.add(wall, wall.wall_coordinates)

    wall = NormalWall(pos_start=(-600, -18),
                      pos_end=(-600, -179),
                      width=wall_thickness)
    playground.add(wall, wall.wall_coordinates)

    wall = NormalWall(pos_start=(-290, 104),
                      pos_end=(105, 104),
                      width=wall_thickness)
    playground.add(wall, wall.wall_coordinates)

    wall = NormalWall(pos_start=(400, 35),
                      pos_end=(400, -42),
                      width=wall_thickness)
    playground.add(wall, wall.wall_coordinates)

    wall = NormalWall(pos_start=(155, -100),
                      pos_end=(-160, -100),
                      width=wall_thickness)
    playground.add(wall, wall.wall_coordinates)