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
    _ego_max_velocity_allowed = 15  # Maximum allowed velocity [m/s]
    # _ego_avg_velocity_expected = 4  # Average expected velocity [m/s]
    # _ego_expected_driven_distance = 70  # Expected driven distance [m]
    _ego_distance_to_traffic_light = 20  # Trigger distance to traffic light [m]
    _ego_distance_to_drive = 50  # Allowed distance to drive, wait the ego vehicle drive for this allowed distance

    # other vehicle
    _other_actor_target_velocity = 7  # Target velocity of other vehicle [m/s]
    _other_actor_max_brake = 1.0  # Maximum brake of other vehicle
    _other_actor_distance = 50  # Distance the other vehicle should drive

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
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation
        )

        first_vehicle = CarlaActorPool.request_new_actor(config.other_actors[0].model, first_vehicle_transform)

        self.other_actors.append(first_vehicle)

    def _create_behavior(self):



        sequence = py_trees.composites.Sequence("Sequence Behavior")


        # Selecting straight path at intersection

        # Generating waypoint list till next intersection

        # adding flow of actors

        # destroying flow of actors

        # move actor

        move_actor = BasicAgentBehavior(self.other_actors[0], carla.Location(x=-80, y=150, z=0), target_speed=30.0,
                                        name="BasicAgentBehavior")



        # wait
        wait = DriveDistance(
            self.ego_vehicles[0],
            self._ego_distance_to_drive,
            name="DriveDistance")

        # behaviour tree
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(wait)
        root.add_child(move_actor)

        ## build the sequence behaviour tree
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        sequence.add_child(root)
        sequence.add_child(ActorDestroy(self.other_actors[0]))





    def _create_test_criteria(self):
        criteria = []

        collision_criteria = CollisionTest(self.ego_vehicle[0])
        criteria.append(collision_criteria)

    def __del__(self):
        self.remove_all_actors()
