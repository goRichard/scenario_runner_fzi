from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *

import re

TEST_SCENARIO = [
    "TestScenario"
]

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


class TestScenario(BasicScenario):
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
        self._goals = [(-42.684, 1.499), (-11.5, 32.658), (4, -40.412), (-6.419, 40.918)]
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        preset = self._weather_presets[self._weather_index]
        world.set_weather(preset[0])

        # Call constructor of BasicScenario
        super(TestScenario, self).__init__(
          "TestScenario",
          ego_vehicles,
          config,
          world,
          debug_mode,
          criteria_enable=criteria_enable)

    def _create_behavior(self):
        """
        3 agents enter roundabout and start at scenario start
        """
        # root for all behaviors
        roundabout_root = py_trees.composites.Sequence()
        scenario_sequence = py_trees.composites.Sequence()
        roundabout_root.add_child(scenario_sequence)

        # ACTOR 1
        # after ego reaches this trigger, first actor accelerates
        start_first_actor_trigger = InTriggerRegion(
            self.ego_vehicles[0],
            -11, -1,
            -35, -25)

        # root for tree of first actor (right entry of roundabout)
        #actor1_tree = py_trees.composites.Parallel(
                #policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        a1_sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicles[0],
            carla.Location(x=-25.4, y=2.9))
        a1_trigger_region = InTriggerRegion(
            self.ego_vehicles[0],
            -21, -11,
            -13, 0)

        scenario_sequence.add_child(start_first_actor_trigger)
        #scenario_sequence.add_child(actor1_tree)

        #actor1_tree.add_child(a1_sync_arrival)
        #actor1_tree.add_child(a1_trigger_region)

        a1_a2_a3 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # parallel to actor 2 start behavior, actor 1 is now basic agent
        # change behavior from sync mode to basic agent
        loc1 = carla.Location(self._goals[1][0], self._goals[1][1], 0.0)
        a1_a2_a3.add_child(BasicAgentBehavior(self.other_actors[0], loc1))

        # ACTOR 2
        actor2_tree = py_trees.composites.Sequence()

        # trigger and behavior for a2
        a2_trigger_region = InTriggerRegion(
            self.ego_vehicles[0],
            -23, 13,
            2, 12)
        actor2_tree.add_child(a2_trigger_region)
        loc2 = carla.Location(self._goals[3][0], self._goals[3][1], 0.0)
        actor2_tree.add_child(BasicAgentBehavior(self.other_actors[1], loc2))
        a1_a2_a3.add_child(actor2_tree)

        # ACTOR 3
        actor3_tree = py_trees.composites.Sequence()
        # trigger and behavior for a3
        a3_trigger_region = InTriggerRegion(
            self.ego_vehicles[0],
            -12, 2,
            16, 26)
        actor3_tree.add_child(a3_trigger_region)
        loc3 = carla.Location(self._goals[0][0], self._goals[0][1], 0.0)
        actor3_tree.add_child(BasicAgentBehavior(self.other_actors[2], loc3))
        a1_a2_a3.add_child(actor3_tree)

        scenario_sequence.add_child(a1_a2_a3)
        return roundabout_root

    def _create_test_criteria(self):
        """
        different test criteria
        """
        parallel_criteria = py_trees.composites.Parallel("group_criteria",
                                                         policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        collision_criterion = CollisionTest(self.ego_vehicles[0], terminate_on_failure=False)
        parallel_criteria.add_child(collision_criterion)

        in_region_criterion = InRadiusRegionTest(self.ego_vehicles[0], self._goals[2][0], self._goals[2][1], 2.0)
        parallel_criteria.add_child(in_region_criterion)

        return parallel_criteria

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.vehicle.get_world().set_weather(preset[0])
        print("weather: " + str(preset[0]))