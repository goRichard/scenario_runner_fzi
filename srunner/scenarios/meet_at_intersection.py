#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Follow leading vehicle scenario:

The scenario realizes a common driving behavior, in which the
user-controlled ego vehicle follows a leading car driving down
a given road. At some point the leading car has to slow down and
finally stop. The ego vehicle has to react accordingly to avoid
a collision. The scenario ends either via a timeout, or if the ego
vehicle stopped close enough to the leading vehicle
"""

import random

import py_trees

import carla

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *


MEET_AT_INTERSECTION_SCENARIO = ["MeetAtIntersection"]


class MeetAtIntersection(BasicScenario):
    category = "MeetAtIntersection"
    timeout = 120  # Timeout of scenario in seconds

    _other_vehicle_target_velocity = 20

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):

        self._map = CarlaDataProvider.get_map()  # get the map instance from carla map class
        self._other_actor_max_brake = 1.0  # brake value for the other actor
        # Timeout of scenario in seconds
        self.timeout = timeout
        super(MeetAtIntersection, self).__init__("MeetAtIntersection",
                                                 ego_vehicles,
                                                 config,
                                                 world,
                                                 debug_mode,
                                                 criteria_enable=criteria_enable)

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
        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        target_location = carla.Location(self.other_actors[0].transform.location.x,
                                         self.other_actors[0].transform.location.y + 200,
                                         self.other_actors[0].transform.location.z)

        move_actor = BasicAgentBehavior(self.other_actors[0], target_location,
                                        self._other_actor_target_speed, name="BasicAgentBehavior")

        # end condition

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(move_actor)
        sequence.add_child(InTriggerDistanceToVehicle(self.other_actors[0],
                                                      self.ego_vehicles[0],
                                                      distance=20,
                                                      name="FinalDistance"))

        sequence.add_child(StandStill(self.other_actors[0], name="StandStill"))
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
