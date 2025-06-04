import numpy as np
from scipy.ndimage import label
import cv2
from spg_overlay.utils.constants import MAX_RANGE_LIDAR_SENSOR
from spg_overlay.utils.grid import Grid
from spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor
from solutions.data_config import *
from solutions.position import Position
from solutions.astar import *

from sklearn.cluster import DBSCAN


class OccupancyGrid(Grid):

    OBSTACLE = GridParams.OBSTACLE
    FREE = GridParams.FREE
    UNDISCOVERED = GridParams.UNDISCOVERED

    class Frontier:
        MIN_FRONTIER_SIZE = GridParams.MIN_FRONTIER_SIZE

        def __init__(self, cells):
            self.cells = cells

        def compute_centroid(self):
            if self.cells.size == 0:
                return None
            x_coords, y_coords = zip(*self.cells)
            return np.array([sum(x_coords) / len(x_coords), sum(y_coords) / len(y_coords)], dtype=int)

        def size(self):
            return len(self.cells)

    def __init__(self,  size_area_world, resolution: float, lidar, semantic):
        super().__init__(size_area_world=size_area_world, resolution=resolution)

        self.size_area_world = size_area_world
        self.resolution = resolution

        self.lidar = lidar

        self.semantic = semantic

        self.x_max_grid: int = int(self.size_area_world[0] / self.resolution + 0.5)
        self.y_max_grid: int = int(self.size_area_world[1] / self.resolution + 0.5)

        self.initial_cell = None
        self.initial_cell_value = None

        WORLD_BORDERS_VALUE = GridParams.WORLD_BORDERS_VALUE
        self.grid = np.zeros((self.x_max_grid, self.y_max_grid))
        self.grid[[0, -1], :] = WORLD_BORDERS_VALUE
        self.grid[:, [0, -1]] = WORLD_BORDERS_VALUE

        self.zoomed_grid = np.empty((self.x_max_grid, self.y_max_grid))

        self.frontier_connectivity_structure = np.ones((3, 3), dtype=int)
        self.frontiers = []

    def set_initial_cell(self, world_x, world_y):
        cell_x, cell_y = self._conv_world_to_grid(world_x, world_y)
        self.initial_cell = (cell_x, cell_y)

    def to_ternary_map(self):
        OBSTACLE_THRESHOLD = GridParams.OBSTACLE_THRESHOLD
        FREE_THRESHOLD = GridParams.FREE_THRESHOLD

        ternary_map = np.zeros_like(self.grid, dtype=int)
        ternary_map[self.grid > OBSTACLE_THRESHOLD] = self.OBSTACLE
        ternary_map[self.grid < FREE_THRESHOLD] = self.FREE
        ternary_map[self.grid == 0] = self.UNDISCOVERED
        return ternary_map

    def update(self, pose: Position):

        LIDAR_DIST_CLIP = GridParams.LIDAR_DIST_CLIP
        MAX_RANGE_LIDAR_SENSOR_FACTOR = GridParams.MAX_RANGE_LIDAR_SENSOR_FACTOR
        EMPTY_ZONE_VALUE = GridParams.EMPTY_ZONE_VALUE
        OBSTACLE_ZONE_VALUE = GridParams.OBSTACLE_ZONE_VALUE
        FREE_ZONE_VALUE = GridParams.FREE_ZONE_VALUE
        THRESHOLD_MIN = GridParams.THRESHOLD_MIN
        THRESHOLD_MAX = GridParams.THRESHOLD_MAX

        lidar_dist = self.lidar.get_sensor_values()[::3].copy()
        lidar_angles = self.lidar.ray_angles[::3].copy()

        # Used to go from rays to points on the grid
        cos_rays = np.cos(lidar_angles + pose.orientation)
        sin_rays = np.sin(lidar_angles + pose.orientation)

        # Any ray that has an associated lidar_dist greater than this threshold is considered to have no obstacle
        no_obstacle_ray_distance_threshold = MAX_RANGE_LIDAR_SENSOR * MAX_RANGE_LIDAR_SENSOR_FACTOR

        # Ensure coherent values to balance noise on lidar values
        processed_lidar_dist = np.clip(lidar_dist - LIDAR_DIST_CLIP, 0, no_obstacle_ray_distance_threshold)
        points_x = pose.position[0] + np.multiply(processed_lidar_dist,
                                                  cos_rays)
        points_y = pose.position[1] + np.multiply(processed_lidar_dist,
                                                  sin_rays)

        for pt_x, pt_y in zip(points_x, points_y):
            self.add_value_along_line(pose.position[0], pose.position[1], pt_x, pt_y, EMPTY_ZONE_VALUE)

        select_collision = lidar_dist < no_obstacle_ray_distance_threshold

        points_x = pose.position[0] + np.multiply(lidar_dist, cos_rays)
        points_y = pose.position[1] + np.multiply(lidar_dist, sin_rays)

        zone_drone_x, zone_drone_y = self.compute_near_drones_zone(pose)
        for ind, v in enumerate(select_collision):
            if select_collision[ind] == True:
                if (self.list_any_comparaison_int(abs(zone_drone_x - points_x[ind]), 3)
                        and self.list_any_comparaison_int(abs(zone_drone_y - points_y[ind]), 3)):
                    select_collision[ind] = False

        points_x = points_x[select_collision]
        points_y = points_y[select_collision]

        self.add_points(points_x, points_y, OBSTACLE_ZONE_VALUE)
        self.add_points(pose.position[0], pose.position[1], FREE_ZONE_VALUE)
        self.grid = np.clip(self.grid, THRESHOLD_MIN, THRESHOLD_MAX)
        self.zoomed_grid = self.grid.copy()

        new_zoomed_size = (int(self.size_area_world[1] * 0.5), int(self.size_area_world[0] * 0.5))
        self.zoomed_grid = cv2.resize(self.zoomed_grid, new_zoomed_size,
                                      interpolation=cv2.INTER_NEAREST)

        self.to_ternary_map()

    def frontiers_update(self):
        ternary_map = self.to_ternary_map()

        diff_x = np.diff(ternary_map, axis=1)
        diff_y = np.diff(ternary_map, axis=0)

        boundaries_x = np.abs(diff_x) == 2
        boundaries_y = np.abs(diff_y) == 2

        # Combinaison des résultats
        boundaries_map = np.pad(boundaries_x, ((0, 0), (0, 1))) | np.pad(boundaries_y, ((0, 1), (0, 0)))

        labeled_array, num_features = label(boundaries_map, self.frontier_connectivity_structure)

        # Extraction des points de chaque frontière
        frontiers = [np.argwhere(labeled_array == i) for i in range(1, num_features + 1)]
        self.frontiers = [self.Frontier(cells) for cells in frontiers if len(cells) >= self.Frontier.MIN_FRONTIER_SIZE]

    def cluster_frontiers_dbscan(self, eps=2, min_samples=3):
        self.frontiers_update()
        all_frontier_cells = []
        for frontier in self.frontiers:
            for cell in frontier.cells:
                all_frontier_cells.append(cell)
        all_frontier_cells = np.array(all_frontier_cells)
        if len(all_frontier_cells) == 0:
            return []

        db = DBSCAN(eps=eps, min_samples=min_samples).fit(all_frontier_cells)
        labels = db.labels_
        clusters = []
        for label_val in set(labels):
            cluster_points = all_frontier_cells[labels == label_val]
            centroid = np.mean(cluster_points, axis=0).astype(int)
            clusters.append({"centroid": centroid,
                             "points": cluster_points,
                             "size": len(cluster_points)
            })
        return clusters

    def delete_frontier_artifacts(self, frontier):
        if frontier is not None:
            for cell in frontier.cells:
                self.grid[cell] = GridParams.FRONTIER_ARTIFACT_RESET_VALUE

    def closest_largest_frontier(self, pose: Position):
        self.frontiers_update()
        if not self.frontiers:
            return None

        pos_drone_grid = np.array(self._conv_world_to_grid(*pose.position))
        frontiers_with_size = [
            (frontier, frontier.compute_centroid(), frontier.size()) for frontier in self.frontiers
        ]

        def interest_measure(frontier_data):
            _, centroid, size = frontier_data
            distance = np.linalg.norm(centroid - pos_drone_grid)
            return distance / (size + 1) ** 2

        closest_frontier, closest_centroid, _ = min(frontiers_with_size, key=interest_measure, default=(None, None))
        return (closest_frontier, closest_centroid)

    def compute_safest_path(self, start_cell, target_cell, max_inflation):
        MAP = self.to_ternary_map()

        for inflation in range(max_inflation, 0, -1):  # Decreasing inflation to find the safest path
            MAP_inflated = inflate_obstacles(MAP, inflation)
            start_x, start_y = next_point_free(MAP_inflated, *start_cell, max_inflation - inflation)
            end_x, end_y = next_point_free(MAP_inflated, *target_cell, max_inflation - inflation)

            path = a_star_search(MAP_inflated, (start_x, start_y), (end_x, end_y))

            if path:
                path_simplified = self.simplify_path(path, MAP_inflated) or [start_cell]
                return [self._conv_grid_to_world(x, y) for x, y in path_simplified]

        return None

    def simplify_path(self, path, MAP):
        path_simplified = simplify_collinear_points(path)
        path_line_of_sight = simplify_by_line_of_sight(path_simplified, MAP)
        return ramer_douglas_peucker(path_line_of_sight, 0.5)

    def compute_near_drones_zone(self, pose: Position):
        detection_semantic = self.semantic.get_sensor_values().copy()
        zone_drone_x = []
        zone_drone_y = []
        for data in detection_semantic:
            if (data.entity_type == DroneSemanticSensor.TypeEntity.DRONE):
                cos_rays = np.cos(data.angle + pose.orientation)
                sin_rays = np.sin(data.angle + pose.orientation)

                zone_drone_x.append(pose.position[0] + np.multiply(data.distance, cos_rays))
                zone_drone_y.append(pose.position[1] + np.multiply(data.distance, sin_rays))
        return zone_drone_x, zone_drone_y

    def list_any_comparaison_int(self, L, i):
        for x in L:
            if x < i: return True
        return False

    def score(self):
        """
        Calcule la proportion de cases déjà explorées (non UNDISCOVERED)
        Retourne un score entre 0 et 1
        """
        # Convertir la grille en carte ternaire (OBSTACLE, FREE, UNDISCOVERED)
        ternary_map = self.to_ternary_map()

        # Compter le nombre total de cellules (excluant les bordures)
        total_cells = (self.x_max_grid - 2) * (self.y_max_grid - 2)

        # Compter le nombre de cellules non découvertes (UNDISCOVERED)
        undiscovered_cells = np.count_nonzero(ternary_map == self.UNDISCOVERED)

        # Calculer la proportion de cellules explorées
        explored_proportion = 1.0 - (undiscovered_cells / total_cells)

        return explored_proportion

    def display(self, grid_to_display, robot_pose: Position, title="grid"):
        """
        Screen display of grid and robot pose,
        using opencv (faster than the matplotlib version)
        robot_pose : [x, y, theta] nparray, corrected robot pose
        """
        # Utiliser la zoomed_grid
        # Conversion de la zoomed_grid en représentation ternaire
        ternary_map = np.zeros_like(self.zoomed_grid, dtype=np.int32)

        # Appliquer les seuils pour créer une représentation ternaire
        OBSTACLE_THRESHOLD = GridParams.OBSTACLE_THRESHOLD
        FREE_THRESHOLD = GridParams.FREE_THRESHOLD

        ternary_map[self.zoomed_grid > OBSTACLE_THRESHOLD] = self.OBSTACLE
        ternary_map[self.zoomed_grid < FREE_THRESHOLD] = self.FREE
        ternary_map[self.zoomed_grid == 0] = self.UNDISCOVERED

        # Création d'une image RGB avec 3 couleurs distinctes
        img = ternary_map.T
        img_color = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

        # Définition des couleurs pour chaque état
        # BGR format (OpenCV)
        UNKNOWN_COLOR = (255, 0, 0)  # Bleu pour les zones inconnues
        FREE_COLOR = (0, 255, 0)  # Vert pour les zones libres
        OCCUPIED_COLOR = (0, 0, 255)  # Rouge pour les zones occupées

        # Application des couleurs selon l'état
        img_color[img == self.UNDISCOVERED] = UNKNOWN_COLOR
        img_color[img == self.FREE] = FREE_COLOR
        img_color[img == self.OBSTACLE] = OCCUPIED_COLOR

        cv2.imshow(title, img_color)
        cv2.waitKey(1)

    def merge_maps(self, other_map):
        """
        Merge the other map into the current map.
        1. Récupérer les indices des cases UNDISCOVERED de self.grid
        2. Pour ces indices, si les cases correspondantes dans other_map ne sont pas UNDISCOVERED,
           copier les valeurs de other_map vers self.grid
        """
        # Convertir self.grid en représentation ternaire
        ternary_current = self.to_ternary_map()

        # Convertir other_map en représentation ternaire
        temp_grid = self.grid.copy()
        self.grid = other_map.copy()
        ternary_other = self.to_ternary_map()
        self.grid = temp_grid  # Restaurer la grille d'origine

        # Récupérer les indices des cases UNDISCOVERED dans la carte actuelle
        undiscovered_indices = np.where(ternary_current == -2)  # 0 = UNDISCOVERED

        # Pour ces indices, vérifier si les cases correspondantes dans other_map ne sont pas UNDISCOVERED
        # et si c'est le cas, copier les valeurs de other_map vers self.grid
        for i, j in zip(undiscovered_indices[0], undiscovered_indices[1]):
            if ternary_other[i, j] != -2:  # Si la case n'est pas UNDISCOVERED dans other_map
                # Attribuer la valeur numérique de other_map à la grille principale
                self.grid[i, j] = other_map[i, j]

