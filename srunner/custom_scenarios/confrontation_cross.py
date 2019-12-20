
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


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


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
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        preset = self._weather_presets[self._weather_index]
        world.set_weather(preset[0])
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 40

        self._other_actor_transform = None
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20

        self.timeout = timeout

        # Call constructor of BasicScenario
        super(ConfrontationCross, self).__init__(
          name="ConfrontationCross",
          ego_vehicles=ego_vehicles,
          config=config,
          world=world,
          debug_mode=False,
          criteria_enable=criteria_enable)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        print(self.other_actors[0], self.ego_vehicles[0])

        # reset its pose to the required one

        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior

        other_loc = self.other_actors[0].get_location()
        loc = carla.Location(other_loc.x+100, other_loc.y, other_loc.z)
        # start and accelerate other vehicle
        behavior_other = py_trees.composites.Parallel("Waiting for end position",
                                                      policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        other_forward = BasicAgentBehavior(self.other_actors[0], loc)
        accelerate_other = AccelerateToVelocity(self.other_actors[0], 1, self._first_vehicle_speed)

        # stop vehicle
        stop = py_trees.composites.Sequence("Sequence Behavior")
        stop_trigger = InTriggerDistanceToVehicle(self.other_actors[0], self.ego_vehicles[0],
                                                  distance=30, name="FinalDistance")
        stop_other = StopVehicle(self.other_actors[0], self._other_actor_max_brake)
        stop.add_children([stop_trigger, stop_other])
        behavior_other.add_children([other_forward, accelerate_other, stop])

        # end condition
        end_condition = StandStill(self.ego_vehicles[0], name="StandStill")

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(behavior_other)

        sequence.add_child(end_condition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        Setup the evaluation criteria for NewScenario
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
