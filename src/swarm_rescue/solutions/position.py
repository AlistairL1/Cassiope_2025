from typing import Tuple
import numpy as np
from spg_overlay.utils.utils import normalize_angle

class Position:
    def __init__(self, position=np.zeros(2, ), orientation=0.0, odometer=[0.0, 0.0, 0.0],
                 previous_position=np.zeros(2, ), previous_orientation=0.0, taille: Tuple[float, float] = (0.0, 0.0)):

        if not isinstance(position, np.ndarray):
            print("type position=", type(position))
            raise TypeError("position must be an instance of np.ndarray")

        if position is None or orientation is None:
            self.gps = False

            xmax = int(taille[0] / 2)
            xmin = -xmax
            ymax = int(taille[1] / 2)
            ymin = -ymax

            self.orientation: float = previous_orientation
            self.orientation += odometer[2]
            self.orientation = normalize_angle(self.orientation)

            self.position: np.array = previous_position
            self.position[0] += np.cos(normalize_angle(odometer[1] + previous_orientation)) * odometer[0]
            self.position[1] += np.sin(normalize_angle(odometer[1] + previous_orientation)) * odometer[0]
            self.position[0] = min(max(self.position[0], xmin), xmax)
            self.position[1] = min(max(self.position[1], ymin), ymax)


        else:
            self.gps = True
            self.position: np.array = position
            self.orientation: float = orientation
