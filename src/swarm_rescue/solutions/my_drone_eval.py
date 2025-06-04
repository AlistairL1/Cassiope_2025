from enum import Enum
from collections import deque

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.utils import circular_mean, normalize_angle
from solutions.grids import *
from solutions.messages import DroneMessage
from solutions.data_config import *

from scipy.optimize import linear_sum_assignment


class MyDroneEval(DroneAbstract):
    class State(Enum):
        WAITING = 1

        SEARCHING_WALL = 2
        FOLLOWING_WALL = 3

        EXPLORING_FRONTIERS = 4

        GRASPING_WOUNDED = 5
        SEARCHING_RESCUE_CENTER = 6
        GOING_RESCUE_CENTER = 7

        SEARCHING_RETURN_AREA = 8
        GOING_RETURN_AREA = 9

    def __init__(self, identifier, misc_data, **kwargs):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)

        # MAPPING
        self.mapping_params = MappingParams()
        self.estimated_pose = Position()
        self.grid = OccupancyGrid(size_area_world=self.size_area, resolution=self.mapping_params.resolution,
                                  lidar=self.lidar(), semantic=self.semantic())

        # POSITION
        self.previous_position = deque(maxlen=1)
        self.previous_position.append((0, 0))
        self.previous_orientation = deque(maxlen=1)
        self.previous_orientation.append(0)

        # STATE INITIALISATION
        self.state = self.State.WAITING
        self.previous_state = self.State.WAITING

        # WAITING STATE
        self.waiting_params = WaitingStateParams()
        self.step_waiting_count = 0

        # GRASPING
        self.grasping_params = GraspingParams()

        # WALL FOLLOWING
        self.wall_following_params = WallFollowingParams()

        # FRONTIER EXPLORATION
        self.explored_all_frontiers = False
        self.next_frontier = None
        self.next_frontier_centroid = None

        # PID PARAMS
        self.pid_params = PIDParams()
        self.past_ten_errors_angle = [0 for _ in range(10)]
        self.past_ten_errors_distance = [0 for _ in range(10)]

        # COM PARAMS
        self.com_params = CommunicationParams()

        # PATH FOLLOWING
        self.path_params = PathParams()
        self.indice_current_waypoint = 0
        self.inital_point_path = (0, 0)
        self.finished_path = True
        self.path = []
        self.path_grid = []

        # LOG PARAMS
        self.timestep_count = 0

        self.wounded_locked = []
        self.other_drones_pos = []

    def define_message_for_all(self):
        message = []

        if self.timestep_count % self.com_params.TIME_INTERVAL == 0:
            msg = DroneMessage(category=DroneMessage.Category.MAPPING, arg={"map": self.grid.grid})
            message.append(msg)
        else :
            msg = DroneMessage(category=DroneMessage.Category.PASS, arg=None)
            message.append(msg)
        if self.state in [self.State.GRASPING_WOUNDED, self.State.SEARCHING_RESCUE_CENTER, self.State.GOING_RESCUE_CENTER]:
            msg = DroneMessage(
                category=DroneMessage.Category.LOCK_WOUNDED,
                arg=(self.identifier, self.estimated_pose.position.tolist()))
            message.append(msg)

        return message

    def communication_management(self):
        self.wounded_locked = []
        self.other_drones_pos = []
        if self.communicator:
            received_messages = self.communicator.received_messages
            for msg in received_messages:
                for drone_msg in msg[1]:
                    if drone_msg.category == DroneMessage.Category.MAPPING :
                        self.grid.merge_maps(drone_msg.arg["map"])
                    if drone_msg.category == DroneMessage.Category.LOCK_WOUNDED:
                        drone_id, position = drone_msg.arg
                        self.wounded_locked.append((drone_id, position))


    def control(self):
        self.timestep_count += 1

        exploration_score = self.grid.score() * 100
        if exploration_score > 99 and self.is_inside_return_area:
            print(f"Mission terminée : {exploration_score:.1f}% d'exploration - Drone dans la zone de retour")
            return {"forward": 0.0, "lateral": 0.0, "rotation": 0.0, "grasper": 0}

        # Mise à jour de la cartographie
        self.mapping(display=self.mapping_params.display_map)
        self.communication_management()

        # Retrieve Sensor Data
        found_wall, epsilon_wall_angle, min_dist = self.process_lidar_sensor(self.lidar())
        found_wounded, found_rescue_center, score_wounded, epsilon_wounded, epsilon_rescue_center, is_near_rescue_center, min_dist_wnd = self.process_semantic_sensor()


        # TRANSITIONS OF THE STATE
        self.state_update(found_wall, found_wounded, found_rescue_center)

        # Execute Corresponding Command
        state_handlers = {
            self.State.WAITING: self.handle_waiting,
            self.State.SEARCHING_WALL: self.handle_searching_wall,
            self.State.FOLLOWING_WALL: lambda: self.handle_following_wall(epsilon_wall_angle, min_dist),
            self.State.GRASPING_WOUNDED: lambda: self.handle_grasping_wounded(min_dist_wnd, epsilon_wounded),
            self.State.SEARCHING_RESCUE_CENTER: self.handle_searching_rescue_center,
            self.State.GOING_RESCUE_CENTER: lambda: self.handle_going_rescue_center(epsilon_rescue_center, is_near_rescue_center),
            self.State.EXPLORING_FRONTIERS: self.handle_exploring_frontiers,
        }
        # print(self.identifier, self.state)

        return state_handlers[self.state]()


    def handle_waiting(self):
        self.next_frontier = None
        self.next_frontier_centroid = None
        self.finished_path = True
        self.path = []

        self.step_waiting_count += 1
        return {"forward": 0.0, "lateral": 0.0, "rotation": 0.0, "grasper": 0}

    def handle_searching_wall(self):
        return {"forward": 0.5, "lateral": 0.0, "rotation": 0.0, "grasper": 0}

    def handle_following_wall(self, epsilon_wall_angle, min_dist):
        epsilon_wall_angle = normalize_angle(epsilon_wall_angle)
        epsilon_wall_distance = min_dist - self.wall_following_params.dist_to_stay

        command = {"forward": self.wall_following_params.speed_following_wall, "lateral": 0.0, "rotation": 0.0,
                   "grasper": 0}
        command = self.pid_controller(command, epsilon_wall_angle, self.pid_params.Kp_angle, self.pid_params.Kd_angle,
                                      self.pid_params.Ki_angle, self.past_ten_errors_angle, "rotation")
        command = self.pid_controller(command, epsilon_wall_distance, self.pid_params.Kp_distance,
                                      self.pid_params.Kd_distance, self.pid_params.Ki_distance,
                                      self.past_ten_errors_distance, "lateral")

        return command

    def handle_grasping_wounded(self, score_wounded, epsilon_wounded):
        epsilon_wounded = normalize_angle(epsilon_wounded)
        print(self.identifier, score_wounded)
        command = {"forward": self.grasping_params.grasping_speed, "lateral": 0.0, "rotation": 0.0,
                   "grasper": 1 if score_wounded < self.grasping_params.grasping_dist else 0}
        return self.pid_controller(command, epsilon_wounded, self.pid_params.Kp_angle, self.pid_params.Kd_angle,
                                   self.pid_params.Ki_angle, self.past_ten_errors_angle, "rotation")

    def handle_searching_rescue_center(self):
        if self.previous_state is not self.State.SEARCHING_RESCUE_CENTER:
            self.plan_path_to_rescue_center()
        return self.follow_path(self.path, found_and_near_wounded=True)

    def plan_path_to_rescue_center(self):
        start_cell = self.grid._conv_world_to_grid(*self.estimated_pose.position)
        target_cell = self.grid.initial_cell
        max_inflation = self.path_params.max_inflation_obstacle
        self.path = self.grid.compute_safest_path(start_cell, target_cell, max_inflation)
        self.indice_current_waypoint = 0

    def handle_going_rescue_center(self, epsilon_rescue_center, is_near_rescue_center):
        epsilon_rescue_center = normalize_angle(epsilon_rescue_center)
        command = {"forward": 3 * self.grasping_params.grasping_speed, "lateral": 0.0, "rotation": 0.0, "grasper": 1}
        command = self.pid_controller(command, epsilon_rescue_center, self.pid_params.Kp_angle,
                                      self.pid_params.Kd_angle, self.pid_params.Ki_angle, self.past_ten_errors_angle,
                                      "rotation")

        if is_near_rescue_center:
            command["forward"] = 0.0
            command["rotation"] = 1.0

        return command

    def handle_exploring_frontiers(self):
        if self.finished_path:
            self.plan_path_to_frontier()
            self.finished_path = False

        if self.explored_all_frontiers or self.path is None:
            return self.handle_waiting()
        else:
            return self.follow_path(self.path, found_and_near_wounded=False)

    def assign_frontier_cluster(self):
        """
        Utilise DBSCAN pour regrouper les points frontaliers et assigner les clusters aux drones.
        """
        clusters = self.grid.cluster_frontiers_dbscan(eps=2, min_samples=3)
        if not clusters:
            return None

        drone_positions = {}
        drone_positions[self.identifier] = np.array(self.estimated_pose.position)
        for drone_id, pos in self.other_drones_pos:
            drone_positions[drone_id] = np.array(pos)

        drone_ids = sorted(drone_positions.keys())
        num_drones = len(drone_ids)
        num_clusters = len(clusters)

        cost_matrix = np.zeros((num_drones, num_clusters))
        for i, drone_id in enumerate(drone_ids):
            drone_pos = drone_positions[drone_id]
            for j, cluster in enumerate(clusters):
                centroid = cluster["centroid"]
                cost_matrix[i, j] = np.linalg.norm(drone_pos - centroid) / (cluster["size"] + 1)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        assignments = {drone_ids[r]: clusters[col_ind[r]] for r in range(len(row_ind))}

        if self.identifier not in assignments:
            drone_index = drone_ids.index(self.identifier)
            min_cluster_index = np.argmin(cost_matrix[drone_index, :])
            assignments[self.identifier] = clusters[min_cluster_index]

        return assignments[self.identifier]

    def plan_path_to_frontier(self):
        if self.grid.closest_largest_frontier(self.estimated_pose) is not None:
            self.next_frontier, self.next_frontier_centroid = self.grid.closest_largest_frontier(self.estimated_pose)
            if self.next_frontier_centroid is not None:
                start_cell = self.grid._conv_world_to_grid(*self.estimated_pose.position)
                target_cell = self.next_frontier_centroid
                max_inflation = self.path_params.max_inflation_obstacle

                self.path = self.grid.compute_safest_path(start_cell, target_cell, max_inflation)
                print(self.path)
                if self.path is None:
                    print(self.next_frontier.cells)
                else:
                    self.indice_current_waypoint = 0

        else:
            self.explored_all_frontiers = True

    def plan_path_to_frontier(self):
        assigned_cluster = self.assign_frontier_cluster()
        if assigned_cluster is not None:
            self.next_frontier_centroid = assigned_cluster["centroid"]
            start_cell = self.grid._conv_world_to_grid(*self.estimated_pose.position)
            target_cell = self.next_frontier_centroid
            max_inflation = self.path_params.max_inflation_obstacle
            self.path = self.grid.compute_safest_path(start_cell, target_cell, max_inflation)
            if self.path is None:
                self.grid.delete_frontier_artifacts(self.next_frontier)
            else:
                self.indice_current_waypoint = 0
        else:
            self.explored_all_frontiers = True

    def process_semantic_sensor(self):
        semantic_values = self.semantic_values()

        best_angle_wounded = 0
        best_angle_rescue_center = 0
        mindist = 1000
        found_wounded = False
        found_rescue_center = False
        is_near_rescue_center = False
        angles_list = []

        scores = []
        for data in semantic_values:
            if (data.entity_type == DroneSemanticSensor.TypeEntity.RESCUE_CENTER):
                found_rescue_center = True
                angles_list.append(data.angle)
                is_near_rescue_center = (data.distance < 30)
                best_angle_rescue_center = circular_mean(np.array(angles_list))

            elif (data.entity_type == DroneSemanticSensor.TypeEntity.WOUNDED_PERSON and not data.grasped):
                found_wounded = True
                v = (data.angle * data.angle) + (data.distance * data.distance / 10 ** 5)
                scores.append((v, data.angle, data.distance))

        filtered_scores = []
        for score in scores:
            conflict = False
            for wnd_locked in self.wounded_locked:
                dx = score[2] * math.cos(score[1] + self.estimated_pose.orientation)
                dy = score[2] * math.sin(score[1] + self.estimated_pose.orientation)
                detection_position = np.array(self.estimated_pose.position) + np.array([dx, dy])
                conflict = False
                if np.linalg.norm(detection_position - np.array(wnd_locked[1])) < 20.0:
                    conflict = True
                    break
            if not conflict:
                filtered_scores.append(score)
        best_score = 10000
        for score in filtered_scores:
            if score[0] < best_score:
                best_score = score[0]
                best_angle_wounded = score[1]
                mindist = score[2]

        return found_wounded, found_rescue_center, best_score, best_angle_wounded, best_angle_rescue_center, is_near_rescue_center, mindist

    def process_lidar_sensor(self, self_lidar):
        lidar_values = self_lidar.get_sensor_values()

        if lidar_values is None:
            return (False, 0)

        ray_angles = self_lidar.ray_angles
        size = self_lidar.resolution

        angle_nearest_obstacle = 0
        if size != 0:
            min_dist = min(lidar_values)
            angle_nearest_obstacle = ray_angles[np.argmin(lidar_values)]

        near_obstacle = False
        if min_dist < self.wall_following_params.dmax:
            near_obstacle = True

        epsilon_wall_angle = angle_nearest_obstacle - np.pi / 2

        return (near_obstacle, epsilon_wall_angle, min_dist)


    def pid_controller(self, command, epsilon, Kp, Kd, Ki, past_ten_errors, mode, command_slow=0.8):

        past_ten_errors.pop(0)
        past_ten_errors.append(epsilon)
        if mode == "rotation":
            epsilon = normalize_angle(epsilon)
            deriv_epsilon = normalize_angle(self.odometer_values()[2])
        elif mode == "lateral":
            deriv_epsilon = -np.sin(self.odometer_values()[1]) * self.odometer_values()[0]
        elif mode == "forward":
            deriv_epsilon = self.odometer_values()[0] * np.cos(self.odometer_values()[1])
        else:
            raise ValueError("Mode not found")

        correction_proportionnelle = Kp * epsilon
        correction_derivee = Kd * deriv_epsilon
        correction_integrale = 0
        correction = correction_proportionnelle + correction_derivee + correction_integrale
        command[mode] = correction
        command[mode] = min(max(-1, correction), 1)

        if mode == "rotation":
            if correction > command_slow:
                command["forward"] = self.wall_following_params.speed_turning

        return command

    def is_near_waypoint(self, waypoint):
        distance_to_waypoint = np.linalg.norm(waypoint - self.estimated_pose.position)
        if distance_to_waypoint < self.path_params.distance_close_waypoint:
            return True
        return False

    def follow_path(self, path, found_and_near_wounded):
        if self.is_near_waypoint(path[self.indice_current_waypoint]):
            self.indice_current_waypoint += 1
            if self.indice_current_waypoint >= len(path):
                self.finished_path = True
                self.indice_current_waypoint = 0
                self.path = []
                self.path_grid = []
                return

        return self.go_to_waypoint(path[self.indice_current_waypoint][0], path[self.indice_current_waypoint][1],
                                   found_and_near_wounded)

    def go_to_waypoint(self, x, y, found_and_near_wounded):
        dx = x - self.estimated_pose.position[0]
        dy = y - self.estimated_pose.position[1]
        epsilon = math.atan2(dy, dx) - self.estimated_pose.orientation
        epsilon = normalize_angle(epsilon)
        command_path = self.pid_controller(
            {"forward": 1, "lateral": 0.0, "rotation": 0.0, "grasper": 1 if found_and_near_wounded else 0}, epsilon,
            self.pid_params.Kp_angle_1, self.pid_params.Kd_angle_1, self.pid_params.Ki_angle,
            self.past_ten_errors_angle, "rotation", 0.5)

        if self.indice_current_waypoint == 0:
            x_previous_waypoint, y_previous_waypoint = self.inital_point_path
        else:
            x_previous_waypoint, y_previous_waypoint = self.path[self.indice_current_waypoint - 1][0], \
            self.path[self.indice_current_waypoint - 1][1]

        epsilon_distance = compute_relative_distance_to_droite(x_previous_waypoint, y_previous_waypoint, x, y,
                                                               self.estimated_pose.position[0],
                                                               self.estimated_pose.position[1])

        command_path = self.pid_controller(command_path, epsilon_distance, self.pid_params.Kp_distance_1,
                                           self.pid_params.Kd_distance_1, self.pid_params.Ki_distance_1,
                                           self.past_ten_errors_distance, "lateral", 0.5)

        return command_path

    def state_update(self, found_wall, found_wounded, found_rescue_center):
        self.previous_state = self.state

        exploration_score = self.grid.score() * 100.0
        exploration_complete = exploration_score > 99

        conditions = {
            "found_wall": found_wall,
            "lost_wall": not found_wall,
            "found_wounded": found_wounded,
            "holding_wounded": bool(self.base.grasper.grasped_entities),
            "lost_wounded": not found_wounded and not self.base.grasper.grasped_entities,
            "found_rescue_center": found_rescue_center,
            "lost_rescue_center": not self.base.grasper.grasped_entities,
            "no_frontiers_left": len(self.grid.frontiers) == 0,
            "waiting_time_over": self.step_waiting_count >= self.waiting_params.step_waiting,
            "exploration_complete": exploration_complete
        }

        STATE_TRANSITIONS = {
            self.State.WAITING: {
                "found_wounded": self.State.GRASPING_WOUNDED,
                "waiting_time_over": self.State.EXPLORING_FRONTIERS,
                "exploration_complete": self.State.SEARCHING_RESCUE_CENTER
            },
            self.State.GRASPING_WOUNDED: {
                "lost_wounded": self.State.WAITING,
                "holding_wounded": self.State.SEARCHING_RESCUE_CENTER
            },
            self.State.SEARCHING_RESCUE_CENTER: {
                "lost_rescue_center": self.State.WAITING,
                "found_rescue_center": self.State.GOING_RESCUE_CENTER
            },
            self.State.GOING_RESCUE_CENTER: {
                "lost_rescue_center": self.State.WAITING
            },
            self.State.EXPLORING_FRONTIERS: {
                "found_wounded": self.State.GRASPING_WOUNDED,
                "no_frontiers_left": self.State.FOLLOWING_WALL,
                "exploration_complete": self.State.SEARCHING_RESCUE_CENTER
            },
            self.State.SEARCHING_WALL: {
                "found_wounded": self.State.GRASPING_WOUNDED,
                "found_wall": self.State.FOLLOWING_WALL,
                "exploration_complete": self.State.SEARCHING_RESCUE_CENTER
            },
            self.State.FOLLOWING_WALL: {
                "found_wounded": self.State.GRASPING_WOUNDED,
                "lost_wall": self.State.SEARCHING_WALL,
                "exploration_complete": self.State.SEARCHING_RESCUE_CENTER
            }
        }

        for condition, next_state in STATE_TRANSITIONS.get(self.state, {}).items():
            if conditions[condition]:
                self.state = next_state
                break

        if self.state != self.previous_state and self.state == self.State.WAITING:
            self.step_waiting_count = 0

    def mapping(self, display=False):
        if self.timestep_count == 1:  # first iterations
            print("Starting control")
            start_x, start_y = self.measured_gps_position()  # never none ?
            print(f"Initial position: {start_x}, {start_y}")
            self.grid.set_initial_cell(start_x, start_y)

        self.estimated_pose = Position(np.asarray(self.measured_gps_position()),
                                       self.measured_compass_angle(), self.odometer_values(), self.previous_position[-1],
                                       self.previous_orientation[-1], self.size_area)

        self.previous_position.append(self.estimated_pose.position)
        self.previous_orientation.append(self.estimated_pose.orientation)

        self.grid.update(pose=self.estimated_pose)

        if display and (self.timestep_count % 5 == 0):  # Afficher tous les 5 pas de temps
            self.grid.display(self.grid.zoomed_grid,
                            self.estimated_pose,
                            title=f"Drone {self.identifier} - Grille d'exploration")
