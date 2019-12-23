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

import carla

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *


