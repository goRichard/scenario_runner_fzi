from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *

import re

SETLEVEL_INTERSECTION_3A = [
    "SetLevelIntersection3A"
]

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


class SetLevelIntersection3A(BasicScenario):
    """
    Test scenario to try out framework
    """

    # some ego vehicle parameters
    # some parameters for the other vehicles

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=True, criteria_enable=True,
                 timeout=60):
        """
        Initialize all parameters required for NewScenario
        """
        # Timeout of scenario in seconds
        self.timeout = timeout

        self._map = CarlaDataProvider.get_map()
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        preset = self._weather_presets[self._weather_index]
        world.set_weather(preset[0])

        # Call constructor of BasicScenario
        super(SetLevelIntersection3A, self).__init__(
          "TestScenario",
          ego_vehicles,
          config,
          world,
          debug_mode,
          criteria_enable=criteria_enable)

    def _create_behavior(self):
        """
        3 agents enter intersection and start at scenario start time
        """
        root = py_trees.composites.Sequence()

        # ACTOR 1
        leaf1 = AccelerateToVelocity(self.other_actors[0], 6.0, 28.0)
        leaf2 = KeepVelocity(self.other_actors[0], 28.0)
        root.add_child(leaf1)
        root.add_child(leaf2)

        return root

    def _create_test_criteria(self):
        """
        different test criteria
        """
        parallel_criteria = py_trees.composites.Parallel("group_criteria",
                                                         policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        collision_criterion = CollisionTest(self.ego_vehicles[0], terminate_on_failure=True)
        parallel_criteria.add_child(collision_criterion)

        in_region_criterion = InRadiusRegionTest(self.ego_vehicles[0], 222.3, -249.64, 2.0)
        parallel_criteria.add_child(in_region_criterion)

        return parallel_criteria

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.vehicle.get_world().set_weather(preset[0])
        print("weather: " + str(preset[0]))