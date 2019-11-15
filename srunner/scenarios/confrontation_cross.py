
from __future__ import print_function

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import re
import time
import weakref

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_criteria import *
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import *

from srunner.scenarios.basic_scenario import BasicScenario

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

CONFRONTATION_CROSS = ['ConfrontationCross']


class ConfrontationCross(BasicScenario):
    """
    Some documentation on NewScenario
    :param world is the CARLA world
    :param ego_vehicles is a list of ego vehicles for this scenario
    :param config is the scenario configuration (ScenarioConfiguration)
    :param randomize can be used to select parameters randomly (optional, default=False)
    :param debug_mode can be used to provide more comprehensive console output (optional, default=False)
    :param criteria_enable can be used to disable/enable scenario evaluation based on test criteria (optional, default=True)
    :param timeout is the overall scenario timeout (optional, default=60 seconds)
    """

    category = "ConfrontationCross"

    timeout = 120            # Timeout of scenario in seconds

    # some ego vehicle parameters
    # some parameters for the other vehicles

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Initialize all parameters required for ConfrontationCross
        """
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._first_vehicle_location = 25

        self.timeout = timeout

        # Call constructor of BasicScenario
        super(ConfrontationCross, self).__init__(
          name="ConfrontationCross",
          ego_vehicles=ego_vehicles,
          config=config,
          world=world,
          debug_mode=True,
          criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        print('reference waypoint {0} and first vehicle location {1}'.format(self._reference_waypoint, self._first_vehicle_location))
        print(first_vehicle_waypoint.transform.rotation)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1 - 500),
            first_vehicle_waypoint.transform.rotation)
        first_vehicle = CarlaActorPool.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        # print('Create first vehicle: ', first_vehicle) # print actor's id and type
        print('First vehicle type: {}'.format(first_vehicle.type_id))
        # print(config.trigger_points[0].location)
        print('First vehicle position: ', first_vehicle.get_location())

        self.other_actors.append(first_vehicle)
        print(self.other_actors[0],
        self.ego_vehicles[0])
        print(len(self.ego_vehicles))
        for ego in self.ego_vehicles:
            print(ego, ego.get_location())

        print('Initiation finished\n')

    def create_behavior(self):
        """
        Setup the behavior for NewScenario
        """
        print('create behavior')



    def create_test_criteria(self):
        """
        Setup the evaluation criteria for NewScenario
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria