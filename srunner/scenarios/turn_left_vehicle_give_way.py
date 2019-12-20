import random

import py_trees

import carla

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *
import sys

TURNING_LEFT_SCENARIOS = ['TurnLeftVehicleGiveWay']


class TurnLeftVehicleGiveWay(BasicScenario):
    """
    This class holds everything required for a scenario,
    in which an other vehicle turning left at the intersection
    and give way to the ego vehicle when it go straightforward
    during the green traffic light

    This is a single ego vehicle scenario
    """
    timeout = 120
    category = "TurnLeftVehicleGiveWay"

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20  # Maximum allowed velocity [m/s]
    # _ego_avg_velocity_expected = 4  # Average expected velocity [m/s]
    # _ego_expected_driven_distance = 70  # Expected driven distance [m]
    _ego_distance_to_traffic_light = 20  # Trigger distance to traffic light [m]
    _ego_vehicle_driven_distance = 105  # Allowed distance to drive, wait the ego vehicle drive for this allowed distance

    # other vehicle
    _other_actor_target_velocity = 10  # Target velocity of other vehicle [m/s]
    _other_actor_max_brake = 1.0  # Maximum brake of other vehicle
    _other_actor_distance = 100  # Distance the other vehicle should drive

    # safety distance
    _safe_distance = 5

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        self.category = "TurnLeftVehicleGiveWay"
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._brake_value = 0.5

        # set timeout in seconds
        self.timeout = timeout

        # super() method from basic scenario
        super(TurnLeftVehicleGiveWay, self).__init__('TurnLeftVehicleGiveWay', ego_vehicles,
                                                     config, world, debug_mode, criteria_enable)

        # set traffic light object for ego_vehicle and other actor
        self._traffic_light = None
        _traffic_light_other = None
        self._traffic_light = CarlaDataProvider.get_next_traffic_light(self.ego_vehicle[0], False)
        _traffic_light_other = CarlaDataProvider.get_next_traffic_light(self.other_actors[0], False)

        if self._traffic_light is None or _traffic_light_other is None:
            print("No traffic light for the given location of the other vehicle found")
            sys.exit(-1)  # system exit code -1

        # set the traffic light to green until the simulation time runs out
        self._traffic_light.set_state(carla.TrafficLightState.Green)
        self._traffic_light.set_green_time(self.timeout)
        _traffic_light_other.set_state(carla.TrafficLightState.Green)
        _traffic_light_other.set_green_time(self.timeout)

    def _initialize_actors(self, config):
        """
        custom initialization
        """
        # get the transform of other actor
        self._other_actor_transform = config.other_actors[0].transform

        # get the transform of first vehicle
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z),
            config.other_actors[0].transform.rotation
        )

        first_vehicle = CarlaActorPool.request_new_actor(config.other_actors[0].model, first_vehicle_transform)

        self.other_actors.append(first_vehicle)

    def _create_behavior(self):

        # Start Condition
        start_condition = InTriggerDistanceToNextIntersection(self.ego_vehicles[0], self._ego_distance_to_traffic_light,
                                                              name="InTriggerDistanceToNextIntersection")
        # Generating waypoint list till next intersection

        turn = -1  # drive straight ahead
        """
        turn = -1, drive turn to left
        turn = 1, drive turn to right
        """
        plan = []

        plan, target_waypoint = generate_target_waypoint_list(
            CarlaDataProvider.get_map().get_waypoint(self.other_actors[0].get_location()), turn)

        wp_choice = target_waypoint.next(5.0)
        while len(wp_choice) == 1:
            target_waypoint = wp_choice[0]
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(5.0)

        # continue driving
        continue_driving = py_trees.composites.Parallel(
            "ContinueDriving",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        continue_driving_waypoints = WaypointFollower(self.other_actors[0], self._other_actor_target_velocity,
                                                      plan=plan)

        continue_driving_distance = DriveDistance(
            self.other_actors[0],
            self._other_actor_distance,
            name="Distance")

        sub_sequence_1 = py_trees.composites.Sequence("sub sequence behaviour")
        situation_1 = InTimeToArrivalToVehicle(self.other_actors[0], self.ego_vehicles[0], self._safe_distance,
                                               name="TriggerDistanceToVehicle")
        other_actor_stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake, name="Stopping")

        sub_sequence_1.add_child(situation_1)
        sub_sequence_1.add_child(other_actor_stop)

        # move actor, using waypointfollowe, still don not understand how to use BasicAgentBehaviour

        # wait
        wait = DriveDistance(
            self.ego_vehicles[0],
            self._ego_vehicle_driven_distance,
            name="DriveDistance")

        # behaviour tree
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        root.add_child(continue_driving_distance)
        root.add_child(continue_driving_waypoints)
        root.add_child(sub_sequence_1)
        # build the sequence behaviour tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        sequence.add_child(start_condition)
        sequence.add_child(root)
        sequence.add_child(wait)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        criteria = []

        collision_criteria = CollisionTest(self.ego_vehicle[0])
        criteria.append(collision_criteria)

    def __del__(self):
        self.remove_all_actors()
