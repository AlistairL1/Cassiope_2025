import math
import random
import sys
from pathlib import Path
from typing import List, Type

# Insert the parent directory of the current file's directory into sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from spg.playground import Playground

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.entities.rescue_center import RescueCenter
from spg_overlay.entities.return_area import ReturnArea
from spg_overlay.entities.sensor_disablers import ZoneType, NoComZone, KillZone
from spg_overlay.entities.wounded_person import WoundedPerson
from spg_overlay.gui_map.closed_playground import ClosedPlayground
from spg_overlay.gui_map.map_abstract import MapAbstract
from spg_overlay.reporting.evaluation import ZonesConfig
from spg_overlay.utils.misc_data import MiscData


class CustomMap02(MapAbstract):
    """
    Carte personnalisée basée sur MyMapIntermediate02 mais avec:
    - 2 victimes placées stratégiquement
    - 2 drones positionnés aux coins opposés de la carte
    """

    def __init__(self, zones_config: ZonesConfig = ()):
        super().__init__(zones_config)
        self._max_timestep_limit = 3000
        self._max_walltime_limit = 120

        # PARAMETERS MAP
        self._size_area = (1200, 500)

        self._return_area = ReturnArea(size=(200, 240))
        self._return_area_pos = ((496, 40), 0)

        self._rescue_center = RescueCenter(size=(200, 80))
        self._rescue_center_pos = ((496, 206), 0)

        self._no_com_zone = NoComZone(size=(330, 500))
        self._no_com_zone_pos = ((-159, 0), 0)

        self._kill_zone = KillZone(size=(68, 360))
        self._kill_zone_pos = ((-484, 0), 0)

        # 2 blessés placés de part et d'autre de la carte
        self._wounded_persons_pos = [        ]
        self._number_wounded_persons = len(self._wounded_persons_pos)
        self._wounded_persons: List[WoundedPerson] = []

        # 2 drones placés aux coins opposés de la carte
        self._number_drones = 2
        self._drones_pos = [
            ((-550, 200), 0),  # Coin supérieur gauche
            ((400, -200), 0)   # Coin inférieur droit
        ]
        self._drones: List[DroneAbstract] = []

    def construct_playground(self, drone_type: Type[DroneAbstract]) -> Playground:
        playground = ClosedPlayground(size=self._size_area)

        playground.add(self._return_area, self._return_area_pos)
        playground.add(self._rescue_center, self._rescue_center_pos)

        self._explored_map.initialize_walls(playground)

        # DISABLER ZONES
        if ZoneType.NO_COM_ZONE in self._zones_config:
            playground.add(self._no_com_zone, self._no_com_zone_pos)

        if ZoneType.KILL_ZONE in self._zones_config:
            playground.add(self._kill_zone, self._kill_zone_pos)

        # POSITIONS OF THE WOUNDED PERSONS
        for i in range(self._number_wounded_persons):
            wounded_person = WoundedPerson(rescue_center=self._rescue_center)
            self._wounded_persons.append(wounded_person)
            pos = (self._wounded_persons_pos[i], 0)
            playground.add(wounded_person, pos)

        # POSITIONS OF THE DRONES
        misc_data = MiscData(size_area=self._size_area,
                           number_drones=self._number_drones,
                           max_timestep_limit=self._max_timestep_limit,
                           max_walltime_limit=self._max_walltime_limit)
        for i in range(self._number_drones):
            drone = drone_type(identifier=i, misc_data=misc_data)
            self._drones.append(drone)
            playground.add(drone, self._drones_pos[i])

        return playground 