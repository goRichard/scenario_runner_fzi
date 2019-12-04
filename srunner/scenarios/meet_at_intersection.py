#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import random

import py_trees

import carla

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *

MEET_AT_INTERSECTION_SCENARIOS = ["MeetAtIntersectionTrial",
                                  "MeetAtIntersectionSync"]


class MeetAtIntersectionTrial(BasicScenario):
    category = "MeetAtIntersection"
    timeout = 120  # Timeout of scenario in seconds

    _other_vehicle_target_velocity = 20

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):

        self._map = CarlaDataProvider.get_map()  # get the map instance from carla map class
        self._other_actor_max_brake = 5.0  # brake value for the other actor
        # Timeout of scenario in seconds
        self.timeout = timeout
        self._traffic_light_other = None
        self._traffic_light_ego = None
        self._goals = [(-120.4, 131.7)]
        super(MeetAtIntersectionTrial, self).__init__("MeetAtIntersection",
                                                      ego_vehicles,
                                                      config,
                                                      world,
                                                      debug_mode,
                                                      criteria_enable=criteria_enable)
        self._traffic_light_other = CarlaDataProvider.get_next_traffic_light(self.other_actors[0], False)
        self._traffic_light_other.set_state(carla.TrafficLightState.Red)
        self._traffic_light_ego = CarlaDataProvider.get_next_traffic_light(self.ego_vehicles[0], False)
        self._traffic_light_ego.set_state(carla.TrafficLightState.Red)



        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # after creating this vehicle and then add this new vehicle as new actors to the other_actors list
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        try:
            first_vehicle = CarlaActorPool.request_new_actor(config.other_actors[0].model, self._other_actor_transform)
        except RuntimeError as r:
            raise r
        first_vehicle.set_transform(first_vehicle_transform)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # to avoid the other actor blocking traffic, it was spawed elsewhere
        # reset its pose to the required one
        '''
        This class contains an atomic behavior to set the transform
        of an actor.

        ActorTransfromSetter(actor, transform)
        '''
        if self.ego_vehicles[0].is_at_traffic_light():
            traffic_light = self.ego_vehicles[0].get_traffic_light()
            if traffic_light == carla.TrafficLightState.Red:
                self._traffic_light_ego.set_state(carla.TrafficLightState.Green)
                # self._traffic_light_other.set_state(carla.TrafficLightState.Green)


        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        target_location = carla.Location(self._goals[0][0], self._goals[0][1],
                                         0.0)  # the target location of other vehicle

        #move_actor = KeepVelocity(self.other_actors[0], 10, distance=50, name="KeepVelocity")


        # drive_behaviour
        drive_behaviour_to_next_intersection = py_trees.composites.Parallel('drive_to_next_intersection',
                                                       policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        move_actor = BasicAgentBehavior(self.other_actors[0], target_location=target_location)
        to_next_intersection_15 = InTriggerDistanceToNextIntersection(self.other_actors[0], distance=15,
                                                                     name="InTriggerDistanceToNextIntersection")



        drive_behaviour_to_next_intersection.add_child(move_actor)
        drive_behaviour_to_next_intersection.add_child(to_next_intersection_15)

        # stop
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake, name="Stopping")

        # end condition

        end_condition = py_trees.composites.Parallel("Waiting for end position",
                                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        end_condition_part1 = StandStill(self.other_actors[0],
                                         name="StandStill_other")  # standstill for other_vehicle
        end_condition_part2 = StandStill(self.ego_vehicles[0], name="StandStill_ego")
        end_condition.add_child(end_condition_part1)
        end_condition.add_child(end_condition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(drive_behaviour_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(end_condition_part1)
        #sequence.add_child(ActorDestroy(self.other_actors[0]))
        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class MeetAtIntersectionSynC(BasicScenario):
    category = "MeetAtIntersection"
    timeout = 120  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):

        self._map = CarlaDataProvider.get_map()  # get the map instance from carla map class
        self._other_actor_max_brake = 1.0  # brake value for the other actor
        # Timeout of scenario in seconds
        self.timeout = timeout
        self._traffic_light = None

        # set the goals for other autos
        self._goals = [(-140.15, 131.7)]
        super(MeetAtIntersectionSynC, self).__init__("MeetAtIntersection",
                                                     ego_vehicles,
                                                     config,
                                                     world,
                                                     debug_mode,
                                                     criteria_enable=criteria_enable)

        # set the traffic light
        self._traffic_light = CarlaDataProvider.get_next_traffic_light(self.ego_vehicles[0], False)
        self._traffic_light.set_state(carla.TrafficLightState.Green)
        self._traffic_light.set_green_time(self.timeout)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)
    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # after creating this vehicle and then add this new vehicle as new actors to the other_actors list
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        try:
            first_vehicle = CarlaActorPool.request_new_actor(config.other_actors[0].model, self._other_actor_transform)
        except RuntimeError as r:
            raise r
        first_vehicle.set_transform(first_vehicle_transform)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):


        # to avoid the other actor blocking traffic, it was spawed elsewhere
        # reset its pose to the required one

        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        target_location = carla.Location(self._goals[0][0], self._goals[0][1],
                                         0.0)  # the target location of other vehicle
        move_actor = BasicAgentBehavior(self.other_actors[0], target_location, name="BasicAgentBehavior")

        distance_to_ego_vehicles = InTriggerDistanceToVehicle(self.other_actors[0], self.ego_vehicles[0], distance=10)

        parallel_drive_behaviour_1 = py_trees.composites.Parallel("DrivingTowardsEgoVehicle",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel_drive_behaviour_1.add_child(move_actor)
        parallel_drive_behaviour_1.add_child(distance_to_ego_vehicles)
        # stop
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake, name="Stopping")

        # end condition

        end_condition = py_trees.composites.Parallel("Waiting for end position",
                                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        end_condition_part1 = StandStill(self.other_actors[0],
                                         name="StandStillOther")  # standstill for other_vehicle
        end_condition_part2 = StandStill(self.ego_vehicles[0], name="StandStillEgo" )
        end_condition.add_child(end_condition_part1)
        end_condition.add_child(end_condition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(parallel_drive_behaviour_1)
        sequence.add_child(stop)
        sequence.add_child(end_condition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
