"""
transitions_ros machine executor node.

August 28, 2025
"""

# Copyright 2025 Intelligent Systems Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import importlib
import sys
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from dua_node_py.dua_node import NodeBase

from std_srvs.srv import SetBool

from transitions_ros.machine import Machine


class FSMExecutorThread(threading.Thread):
    """Stoppable thread that executes a transitions_ros state machine."""
    def __init__(self, fsm: Machine, *args, **kwargs):
        """Constructor."""
        super().__init__(*args, **kwargs)
        self._fsm = fsm
        self._stop_event = threading.Event()

    def stop(self):
        """Sets the stop event, requesting executor termination."""
        self._stop_event.set()

    def run(self):
        """Runs the thread, executing the state machine."""
        while not self._stop_event.is_set():
            self._fsm.run()


class MachineExecutorNode(NodeBase):
    """
    Node that executes a state machine compatible with the transitions_ros package.
    It contains:
      - a dedicated for machine.run() execution;
      - an enable/disable service;
      - possibility to load a state machine in terms of state and transition tables stored in node parameters;
      - possibility to load state routines from a module, specifying its name as a node parameter;
      - 'blackboard' dictionary accessible to all routines, optionally persistent between multiple FSM runs;
      - an 'entities_map' dictionary for registering topic publishers, service/action clients, avoiding duplication.
    """

    def __init__(self, node_name: str, verbose: bool = True):
        """
        Constructor.

        :param node_name: Name of the node.
        :param verbose: Verbosity flag.
        """
        super().__init__(node_name, verbose)

        # Initialize internal state variables
        self._blackboard = {}
        self._executor: FSMExecutorThread = None
        self.entities_map = {}

        self.get_logger().info("Node initialized")

        # Start FSM execution, if requested
        autostart = self.get_parameter(
            "autostart").get_parameter_value().bool_value
        if autostart:
            self._start_executor()

    def __del__(self):
        """Destructor."""
        self._stop_executor()

    def init_cgroups(self):
        """Initialize callback groups for the node."""
        self._enable_cgroup = MutuallyExclusiveCallbackGroup()

    def init_service_servers(self):
        """Initialize service servers for the node."""
        self._enable_srv = self.dua_create_service_server(
            SetBool,
            "~/enable",
            self._enable_clbk,
            self._enable_cgroup
        )

    def _enable_clbk(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """
        Enable/disable the state machine execution.

        :param req: service request
        :param res: service response
        :return: service response
        """
        if req.data:
            if self._executor is None or not self._executor.is_alive():
                success = self._start_executor()
                res.success = success
                if not success:
                    res.message = "Failed to start FSM executor"
                else:
                    res.message = ""
                return res
        else:
            if self._executor is not None and self._executor.is_alive():
                self._stop_executor()
        res.success = True
        res.message = ""
        return res

    def _stop_executor(self):
        """Stop FSM executor thread."""
        if self._executor is not None and self._executor.is_alive():
            self._executor.stop()
            self._executor.join()
