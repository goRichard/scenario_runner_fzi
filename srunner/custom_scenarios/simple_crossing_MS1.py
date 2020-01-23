#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Simple Pedestrian Crossing Road Scenario:
This Scenario realises the user controls the ego vehicle
turning right at an intersection without traffic light while
a pedestrian walk cross the road at the same time
"""

import py_trees
from functools import *
import carla

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *

SIMPLE_CROSSING_SCENARIOS = [
    "VehicleConfrontationCross"
]


def get_spawn_point(world):
    spawn_points = []
    spawn_points_location = []
    for i in range(50):
        spawn_point = carla.Transform() # carla.Transform contains location and rotation
        spawn_point.location = world.get_random_location_from_navigation()
        if spawn_point.location is not None:
            spawn_points.append(spawn_point)
            spawn_points_location.append([spawn_point.location.x, spawn_point.location.y, spawn_point.location.z])
    return spawn_points, spawn_points_location


def in_radius_check(trigger_point, check_point, radius=50):
    """

    :param radius:int the radius of  circle area around ego vehicle
    :param trigger_point: carla.location
    :param check_point: carla.transform
    :return: the point inside the ego vehicle
    """
    squared_distance = \
        (trigger_point.x - check_point.location.x) ** 2 + (trigger_point.y - check_point.location.y) ** 2 + (
                trigger_point.z - check_point.location.z) ** 2

    euclidean_distance = np.sqrt(squared_distance)

    if euclidean_distance < radius:
        return check_point


def wp_next_direction(waypoint, direction):
    if direction:
        wp_next = waypoint.get_right_lane()
    else:
        wp_next = waypoint.get_left_lane()

    return wp_next


def get_opponent_transform(_start_distance, waypoint, trigger_location, last_waypoint_lane, approach_type):
    """
    calculate the transform of adversary
    :param approach_type: string, to specify which direction cylicst will choose
    :param _start_distance: float, the distance from former point
    :param waypoint:
    :param trigger_location:
    :param last_waypoint_lane:
    :return:
    """

    near_side_offset = {"orientation": 270, "position": [0, 30, 45, 60, 90], "z": 0.25, "k": 1.0}
    far_side_offset = {"orientation": -90, "position": 90, "z": 0.25, "k": 1.0}
    # Returns a list of Waypoints at a certain approximate distance from the current Waypoint,
    # taking into account the shape of the road and its possible deviations, without performing any lane change.
    # The list may be empty if the road ends before the specified distance,
    # for instance, a lane ending with the only option of incorporating to another road.
    _wp = waypoint.next(_start_distance)
    #_wp_list = [wp.transform.location.x for wp in _wp]
    #print(f"_wp = {_wp}")
    if _wp:
        _wp = _wp[-1] # get the last waypoint of _wp waypoint list
    else:
        raise RuntimeError("Cannot get next waypoint !")

    if last_waypoint_lane == carla.LaneType.Shoulder:
        lane_width = 2.5
    elif last_waypoint_lane == carla.LaneType.Sidewalk:
        lane_width = 2.5
    else:
        lane_width = 4.0

    # tune the parameter + or - here to realize the far side and near side function
    # + : location plus offset location, which means the cyclist approach the ego vehicle from nearside
    # - : location minus offset location, which means the cylist approach the ego vehicle from farside
    # original _wp.transform.rotation: (pitch=0, yaw=360, roll=0)
    if approach_type == "farside":
        location = _wp.transform.location  # the last waypoint's location

        # tune this parameter can get different cylist driving direction
        orientation_yaw = _wp.transform.rotation.yaw + far_side_offset["orientation"]

        # tune the distance between cyclist and shoulder (shoulder in this simple case) or sidewalk here
        position_yaw = _wp.transform.rotation.yaw + far_side_offset["position"]

        # in this way by tuning the parameter k and postion_yaw, location of adversary can be varied
        # from cos * k-times lane_width or sin * k-times lane-width
        # offset_location: (0, 2.5)
        offset_location = carla.Location(
            far_side_offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            far_side_offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location -= offset_location
        print(f"far side location = {location}")
        location.z = trigger_location.z + near_side_offset["z"]
        transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        return transform
    elif approach_type == "nearside":
        location = _wp.transform.location
        orientation_yaw = _wp.transform.rotation.yaw + near_side_offset["orientation"]
        position_yaw = _wp.transform.rotation.yaw + near_side_offset["position"][1]

        offset_location = carla.Location(
            near_side_offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            near_side_offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        print(f"near side location: {location}")
        location.z = trigger_location.z + near_side_offset["z"]
        transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        return transform


class VehicleConfrontationCross(BasicScenario):

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._world = world

        # Timeout of scenarios
        self.timeout = timeout

        # other actors parameter
        self._other_actor_target_velocity = 10
        self.category = "SimpleCrossing"
        self._wmap = CarlaDataProvider.get_map()

        ## reference waypoint is where the ego car starts
        # waypoint object is an subclass from world.get_map(), i.e. map class
        # waypoint = map.get_waypoint(vehicle.get_location()), get_waypoint recieve parameters carla.location
        # lane_type = waypoint.lane_type
        # lane_change = waypoint.lane_change

        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        self._trigger_location = config.trigger_points[0].location
        self._other_actor_transform = None
        #self._blueprintWalkers = world.get_blueprint_library().filter("walker.*")  # create the walker object

        # get the spawn points:
        self._spawn_points = world.get_map().get_spawn_points()
        self._spawn_points_location = [spawn_point.location for spawn_point in self._spawn_points]
        self._spawn_points_location_list = [[spawn_point.location.x, spawn_point.location.y, spawn_point.location.z] for
                                            spawn_point in self._spawn_points]
        print(f"spawn_points = {len(self._spawn_points_location)}")  # At 373 points in world walker can be spawned

        # get the spawn poins near trigger points
        in_radius_check_fixed_trigger = partial(in_radius_check, self._trigger_location)
        temp_spawn_points_near_trigger_points = list(map(in_radius_check_fixed_trigger, self._spawn_points))
        self._spawn_points_near_trigger_points = [spawn_point_near_trigger_point for spawn_point_near_trigger_point
                                                  in temp_spawn_points_near_trigger_points
                                                  if spawn_point_near_trigger_point is not None]
        print(f"spawn points near trigger = {self._spawn_points_near_trigger_points}")

        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 6
        # Number of attempts made so far
        self._spawn_attempted = 0

        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        self._num_lane_changes = 0

        super(VehicleConfrontationCross, self).__init__("VehicleTurningRight",
                                                        ego_vehicles,
                                                        config,
                                                        world,
                                                        debug_mode,
                                                        criteria_enable=criteria_enable)

    def _initialize_actors(self, config):

        # waypoint at reference position, the trigger position, where simulation starts
        waypoint = self._reference_waypoint
        # generate_targe_waypoint(waypoint, turn) with distance gap 1 meter
        #  turn = 0 -> straight, -1 -> Left, 1 -> Right
        # This method follow waypoints to a junction and choose path based on turn input
        # usually it will only return the last waypoint
        # make a change to return all the possible points from start point to end point
        waypoint_list = generate_target_waypoint(waypoint, 1)
        print(f"waypoint_list = {len(waypoint_list)}")
        # take the last point as waypoint
        # tune the distance between the cyclist and the beginning of intersection (3m -> 10 m)
        waypoints = waypoint_list[-7:-1]

        waypoint = waypoints[-1]

        _start_distance = 8

        while True:
            # Generates a Waypoint at the center of the right lane based on the direction of the current Waypoint,
            # regardless if the lane change is allowed in this location.
            # 1 for right lane
            # -1 for left lane
            wp_next = waypoint.get_right_lane()
            wp_next_left = waypoint.get_left_lane()
            print(f"wp_next_left = {wp_next_left}")
            self._num_lane_changes += 1

            if wp_next is not None:
                _start_distance += 1
                waypoint = wp_next
                if waypoint.lane_type == carla.LaneType.Shoulder or waypoint.lane_type == carla.LaneType.Sidewalk:
                    last_waypoint_lane = waypoint.lane_type
                    break
            else:
                last_waypoint_lane = waypoint.lane_type
                break

        while True:
            try:
                self._other_actor_transform = get_opponent_transform(_start_distance, waypoint,
                                                                     self._trigger_location, last_waypoint_lane,
                                                                     "nearside")
                first_vehicle = CarlaActorPool.request_new_actor('vehicle.bh.crossbike',
                                                                 self._other_actor_transform)
                first_vehicle.set_simulate_physics(enabled=False)

                break
            except RuntimeError as r:
                # In the case there is an object just move a little bit and retry
                print("Base transform is blocking objects ", self._other_actor_transform)
                _start_distance += 0.2
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r
        # Set the transform to -500 z after we are able to spawn it
        actor_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        first_vehicle.set_transform(actor_transform)
        self.other_actors.append(first_vehicle)
        # self.other_actors.append(self._walker_bp)

    def _create_behavior(self):

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.10 * lane_width * self._num_lane_changes)

        if self._ego_route is not None:
            trigger_distance = InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0], self._ego_route,
                                                                     self.other_actors[0].get_location(), 20)

        else:
            trigger_distance = InTriggerDistanceToVehicle(self.other_actors[0], self.ego_vehicles[0], 20)

        actor_velocity = KeepVelocity(self.other_actors[0], self._other_actor_target_velocity)
        actor_traverse = DriveDistance(self.other_actors[0], 0.30 * lane_width)
        post_timer_velocity_actor = KeepVelocity(self.other_actors[0], self._other_actor_target_velocity)
        post_timer_traverse_actor = DriveDistance(self.other_actors[0], 0.70 * lane_width)
        end_condition = TimeOut(5)

        # non leaf nodes
        scenario_sequence = py_trees.composites.Sequence()
        actor_ego_sync = py_trees.composites.Parallel(
            "Synchronization of actor and ego vehicle",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        after_timer_actor = py_trees.composites.Parallel(
            "After timout actor will cross the remaining lane_width",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # building the tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform,
                                                         name='TransformSetterTS4'))
        scenario_sequence.add_child(trigger_distance)
        scenario_sequence.add_child(actor_ego_sync)
        actor_ego_sync.add_child(actor_velocity)
        actor_ego_sync.add_child(actor_traverse)

        scenario_sequence.add_child(after_timer_actor)
        after_timer_actor.add_child(post_timer_velocity_actor)
        after_timer_actor.add_child(post_timer_traverse_actor)

        scenario_sequence.add_child(end_condition)
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []
        # it functions when the ego vehicle hits the other actors
        collision_criterion = CollisionTest(self.ego_vehicles[0], terminate_on_failure=True)
        # try to test in_region_criterion
        in_region_criterion = InRadiusRegionTest(self.ego_vehicles[0], 302, -246.8, 3.0)

        criteria.append(collision_criterion)
        criteria.append(in_region_criterion)
        return criteria

    def __del__(self):
        """
        remove all the actors
        """
        self.remove_all_actors()
