import math
import random
import sys
from pathlib import Path
from typing import List, Type
import numpy as np

# Insert the parent directory of the current file's directory into sys.path.
# This allows Python to locate modules that are one level above the current
# script, in this case spg_overlay.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from spg.playground import Playground

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.drone_motionless import DroneMotionless
from spg_overlay.entities.rescue_center import RescueCenter
from spg_overlay.entities.return_area import ReturnArea
from spg_overlay.entities.wounded_person import WoundedPerson
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.reporting.evaluation import ZonesConfig
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.pose import Pose

from maps.walls_gotta_hurry_02 import add_walls, add_boxes


class MapMaze(MapAbstract):

    def __init__(self, zones_config: ZonesConfig = ()):
        super().__init__(zones_config)
        self._max_timestep_limit = 4000
        self._max_walltime_limit = 240  # In seconds

        # PARAMETERS MAP
        self._size_area = (1600, 1000)

        # Return area (where drones start)
        self._return_area = ReturnArea(size=(200, 200))
        self._return_area_pos = ((-600, 400), 0)

        # Rescue center
        self._rescue_center = RescueCenter(size=(100, 200))
        self._rescue_center_pos = ((-750, 400), 0)

        # Wounded persons positions (one at each end of the maze)
        self._wounded_persons_pos = [(0, 0)]
        self._number_wounded_persons = len(self._wounded_persons_pos)
        self._wounded_persons: List[WoundedPerson] = []

        # POSITIONS OF THE DRONES
        # Place drones in the return area
        self._drones_pos = [((-600, 400), 0)]
        self._number_drones = len(self._drones_pos)
        self._drones: List[DroneAbstract] = []

    def construct_playground(self, drone_type: Type[DroneAbstract]) -> Playground:
        playground = ClosedPlayground(size=self._size_area)

        # Add return area and rescue center
        playground.add(self._return_area, self._return_area_pos)
        playground.add(self._rescue_center, self._rescue_center_pos)

        # Add walls from the walls_gotta_hurry_02.py file
        add_walls(playground)
        add_boxes(playground)

        self._explored_map.initialize_walls(playground)

        # Add wounded persons
        for i in range(self._number_wounded_persons):
            wounded_person = WoundedPerson(rescue_center=self._rescue_center)
            self._wounded_persons.append(wounded_person)
            pos = (self._wounded_persons_pos[i], 0)
            playground.add(wounded_person, pos)

        # Add drones
        misc_data = MiscData(size_area=self._size_area,
                             number_drones=self._number_drones,
                             max_timestep_limit=self._max_timestep_limit,
                             max_walltime_limit=self._max_walltime_limit)
        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            playground.add(drone, self._drones_pos[i])

        return playground


if __name__ == '__main__':
    my_map = MapMaze()
    my_playground = my_map.construct_playground(drone_type=DroneMotionless)

    gui = GuiSR(playground=my_playground,
                the_map=my_map,
                use_mouse_measure=True,
                )
    gui.run() 