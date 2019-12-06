#!/usr/bin/env python

# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the Scenario and ScenarioManager implementations.
These must not be modified and are for reference only!
"""

from __future__ import print_function
import sys
import time
import threading

import py_trees

import numpy as np
import csv
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.result_writer import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime, TimeOut

from pynput.keyboard import Key, Controller


class Scenario(object):
    """
    Basic scenario class. This class holds the behavior_tree describing the
    scenario and the test criteria.

    The user must not modify this class.

    Important parameters:
    - behavior: User defined scenario with py_tree
    - criteria_list: List of user defined test criteria with py_tree
    - timeout (default = 60s): Timeout of the scenario in seconds
    - terminate_on_failure: Terminate scenario on first failure
    """

    def __init__(self, behavior, criteria, name, timeout=60, terminate_on_failure=False):
        self.behavior = behavior
        self.test_criteria = criteria
        self.timeout = timeout

        if self.test_criteria is not None and not isinstance(self.test_criteria, py_trees.composites.Parallel):
            # list of nodes
            for criterion in self.test_criteria:
                criterion.terminate_on_failure = terminate_on_failure

            # Create py_tree for test criteria
            self.criteria_tree = py_trees.composites.Parallel(name="Test Criteria")
            self.criteria_tree.add_children(self.test_criteria)
            self.criteria_tree.setup(timeout=1)
        else:
            self.criteria_tree = criteria

        # Create node for timeout
        self.timeout_node = TimeOut(self.timeout, name="TimeOut")

        # Create overall py_tree
        self.scenario_tree = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        self.scenario_tree.add_child(self.behavior)
        self.scenario_tree.add_child(self.timeout_node)
        if criteria is not None:
            self.scenario_tree.add_child(self.criteria_tree)
        self.scenario_tree.setup(timeout=1)

    def terminate(self):
        """
        This function sets the status of all leaves in the scenario tree to INVALID
        """
        # Get list of all leaves in the tree
        node_list = [self.scenario_tree]
        more_nodes_exist = True
        while more_nodes_exist:
            more_nodes_exist = False
            for node in node_list:
                if node.children:
                    node_list.remove(node)
                    more_nodes_exist = True
                    for child in node.children:
                        node_list.append(child)

        # Set status to INVALID
        for node in node_list:
            node.terminate(py_trees.common.Status.INVALID)


class ScenarioManager(object):
    """
    Basic scenario manager class. This class holds all functionality
    required to start, and analyze a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.execute()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. Trigger a result evaluation with manager.analyze()
    5. Cleanup with manager.stop_scenario()
    """

    def __init__(self, world, debug_mode=False):
        """
        Init requires scenario as input
        """
        self.start_time = None
        self.scenario = None
        self.scenario_tree = None
        self.scenario_class = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self.agent = None
        self._autonomous_agent_plugged = False
        self._running = False
        self._timestamp_last_run = 0.0
        self._my_lock = threading.Lock()

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.shortest_distance = 1000.0
        self._relative_velocity = []
        self._timestamp = []  # create a timestamp list for storing each time stamp

        self.keyboard = Controller()

        self.ego_vehicles_id = []
        self.other_actors_id = []
        self._ego_vehicles_parameters = []
        self._other_vehicles_parameters = []


        world.on_tick(self._tick_scenario)

    def load_scenario(self, scenario):
        """
        Load a new scenario
        """
        self.restart()
        self.scenario_class = scenario
        self.scenario = scenario.scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors
        self.ego_vehicles_id = [ego_vehicle.id for ego_vehicle in self.ego_vehicles]
        self.other_actors_id = [other_actor.id for other_actor in self.other_actors]

        print("ego vehicles id: {}\n".format(self.ego_vehicles_id))
        print("other actors id: {}\n".format(self.other_actors_id))

        CarlaDataProvider.register_actors(self.ego_vehicles)
        CarlaDataProvider.register_actors(self.other_actors)
        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

    def restart(self):
        """
        Reset all parameters
        """
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        GameTime.restart()

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()  # start_game_time = 0.0
        self._running = True

        # create new file
        self.file = open("data_" + str(int(self.start_system_time *1000))+".csv", "a", newline="")
        self.writer = csv.writer(self.file, delimiter=',')
        self.writer.writerow(["Timestamp", "id_ego_vehicles", "id_other_actors"])


        while self._running:
            time.sleep(0.5)

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()
        self.scenario_duration_system = self.end_system_time - \
                                        self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time

        print("scenarion duraiton system: {}".format(self.scenario_duration_system))
        print("scenarion duraiton game: {}".format(self.scenario_duration_game))
        print("time stamp list: {}".format(self._timestamp))
        print("time stamp list length {}".format(len(self._timestamp)))

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario
        This function is a callback for world.on_tick()

        Important:
        - It hast to be ensured that the scenario has not yet completed/failed
          and that the time moved forward.
        - A thread lock should be used to avoid that the scenario tick is performed
          multiple times in parallel.
        """


        with self._my_lock:
            if self._running and self._timestamp_last_run < timestamp.elapsed_seconds:
                self._timestamp_last_run = timestamp.elapsed_seconds

                tick_time = int(time.time() * 1000)  # get every tick seconds with timestamp
                self._timestamp.append(tick_time)  # store the time stamp in list

                if self._debug_mode:
                    print("\n--------- Tick ---------\n")

                # Update game time and actor information
                GameTime.on_carla_tick(timestamp)  # GameTime.on_carla_tick(timestamp) = None
                CarlaDataProvider.on_carla_tick()

                # Tick scenario
                self.scenario_tree.tick_once()  # None

                if self._debug_mode:
                    print("\n")
                    py_trees.display.print_ascii_tree(
                        self.scenario_tree, show_status=True)
                    sys.stdout.flush()

                if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                    self._running = False
                distances = []
                for i in range(len(self.other_actors)):
                    distance = np.sqrt(
                        (self.other_actors[i].get_location().x - self.ego_vehicles[0].get_location().x) ** 2 + (
                                self.other_actors[i].get_location().y - self.ego_vehicles[0].get_location().y) ** 2)
                    distances.append(distance)
                    # print('actor ' + str(i) + ': ' + str(distance))
                if distances[i] < self.shortest_distance:
                    self.shortest_distance = distances[i]
                self.writer.writerow([tick_time, self.ego_vehicles[0].get_transform().pitch,
                                     self.ego_vehicles[0].get_transform().yaw,
                                      self.ego_vehicles[0].get_transform().roll,
                                      self.ego_vehicles[0].get_velocity().x,
                                      self.ego_vehicles[0].get_velocity().y,
                                      self.ego_vehicles[0].get_velocity().z,
                                      self.ego_vehicles[0].get_angular_velocity().x,
                                      self.ego_vehicles[0].get_angular_velocity().y,
                                      self.ego_vehicles[0].get_angular_velocity().z,
                                      self.ego_vehicles[0].get_acceleration().x,
                                      self.ego_vehicles[0].get_acceleration().y,
                                      self.ego_vehicles[0].get_acceleration().z])

    def stop_scenario(self):
        """
        This function triggers a proper termination of a scenario
        """
        if self.scenario is not None:
            self.scenario.terminate()

        # notify agent with escape key to close actual window
        self.keyboard.press(Key.esc)
        self.keyboard.release(Key.esc)
        print("ego vehicles position: {}".format(self._ego_vehicles_position))
        print("ego vehicles velocity: {}".format(self._ego_vehicles_velocity))
        # write recorded data to file and create new recording file every 5 seconds
        # the frequency of creating a new data file denpends on the epoch parameters


        CarlaDataProvider.cleanup()

    def analyze_scenario(self, stdout, filename, junit):
        """
        This function is intended to be called from outside and provide
        statistics about the scenario (human-readable, in form of a junit
        report, etc.)
        """

        failure = False
        timeout = False
        result = "SUCCESS"

        if self.scenario.test_criteria is None:
            return True

        if isinstance(self.scenario.test_criteria, py_trees.composites.Parallel):
            if self.scenario.test_criteria.status == py_trees.common.Status.FAILURE:
                failure = True
                result = "FAILURE"
        else:
            for criterion in self.scenario.test_criteria:
                if (not criterion.optional and
                        criterion.test_status != "SUCCESS" and
                        criterion.test_status != "ACCEPTABLE"):
                    failure = True
                    result = "FAILURE"
                elif criterion.test_status == "ACCEPTABLE":
                    result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit)
        output.write()

        return failure or timeout
