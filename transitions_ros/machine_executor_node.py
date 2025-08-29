"""
transitions_ros machine executor node.

dotX Automation s.r.l. <info@dotxautomation.com>

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

import gc
import importlib
import sys
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from dua_node_py.dua_node import NodeBase

from std_msgs.msg import String

from std_srvs.srv import SetBool

from transitions_ros.machine import Machine, MachineStatus


class FSMExecutorThread(threading.Thread):
    """Stoppable thread that executes a transitions_ros state machine."""

    def __init__(
        self,
        node: NodeBase,
        fsm: Machine,
        wait_servers: bool,
        timeout_sec: float,
        *args,
        **kwargs
    ):
        """
        Constructor.

        :param node: ROS 2 node.
        :param fsm: State machine to execute.
        :param wait_servers: Wait for service/action servers to be available (flag passed to state routines).
        :param timeout_sec: Timeout for service/action client operations (passed to state routines).
        """
        super().__init__(*args, **kwargs)
        self._node = node
        self._fsm = fsm
        self._wait_servers = wait_servers
        self._timeout_sec = timeout_sec
        self._stop_event = threading.Event()

    def stop(self):
        """Sets the stop event, requesting executor termination."""
        self._stop_event.set()

    def run(self):
        """Runs the thread, executing the state machine."""
        self._node.get_logger().warn("FSM executor started")
        curr_status = MachineStatus.IDLE
        while curr_status != MachineStatus.COMPLETED and not self._stop_event.is_set():
            curr_status = self._fsm.run_step(
                wait_servers=self._wait_servers,
                timeout_sec=self._timeout_sec
            )
        if curr_status == MachineStatus.IDLE or curr_status == MachineStatus.RUNNING:
            self._node.get_logger().warn("FSM execution halted")
        elif curr_status == MachineStatus.COMPLETED:
            self._node.get_logger().warn("FSM execution completed")
        else:
            self._node.get_logger().error("FSM execution in unknown state")


class MachineExecutorNode(NodeBase):
    """
    Node that executes a state machine compatible with the transitions_ros package.
    It contains:
      - a dedicated for machine.run() execution;
      - an enable/disable service;
      - possibility to load a state machine in terms of state and transition tables stored in node parameters;
      - possibility to load state routines from a module, specifying its name as a node parameter;
      - 'blackboard' dictionary accessible to all routines, optionally persistent between multiple FSM runs;
      - 'entities_map' dictionary for registering topic publishers, service/action clients, avoiding duplication.
    """

    _PARAMS_FILE_PATH = "/opt/ros/dua-utils/install/share/transitions_ros/machine_executor_params.yaml"

    def __init__(self, node_name: str, verbose: bool = True):
        """
        Constructor.

        :param node_name: Name of the node.
        :param verbose: Verbosity flag.
        """
        super().__init__(node_name, verbose)

        # Initialize internal state variables
        self._executor: FSMExecutorThread = None
        self._fsm: Machine = None
        self._routines_module = None
        self._routines_module_name = ""
        self.blackboard = {}
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

    def init_publishers(self):
        """Initialize publishers for the node."""
        # fsm_state
        self._fsm_state_pub = self.dua_create_publisher(
            String,
            "~/fsm_state"
        )

    def init_service_servers(self):
        """Initialize service servers for the node."""
        self._enable_srv = self.dua_create_service_server(
            SetBool,
            "~/enable",
            self._enable_clbk,
            self._enable_cgroup
        )

    def publish_machine_state(self, state: str):
        """
        Publish the current state of the machine (either a state name or anything else) on the state topic.

        :param state: State (string) to publish.
        """
        self._fsm_state_pub.publish(String(data=state))
        self.get_logger().warn(state)

    def _enable_clbk(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """
        Enable/disable the state machine execution.

        :param req: Service request.
        :param res: Service response.
        :returns: Service response.
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

    def _start_executor(self) -> bool:
        """
        Starts FSM executor thread.

        :returns: True if the executor was started successfully, False otherwise.
        """
        # Reap previous workers
        if self._executor is not None:
            self._stop_executor()

        # Load routines module
        self._routines_module_name = self.get_parameter(
            "transitions.routines_module").get_parameter_value().string_value
        if len(self._routines_module_name) == 0:
            self.get_logger().fatal("Invalid routines module name")
            return False
        try:
            self._routines_module = importlib.import_module(
                self._routines_module_name)
        except ImportError as e:
            self.get_logger().fatal(
                f"Failed to load routines module '{self._routines_module_name}': {e}")
            self._routines_module_name = ""
            return False
        self.get_logger().info(
            f"Loaded routines module: {self._routines_module_name}")

        # Build transitions table
        ok = True
        tran_table = []
        tran_array: list[str] = self.get_parameter(
            "transitions.transitions_table").get_parameter_value().string_array_value
        for tr in tran_array:
            tr_elements = tr.split(";", 2)
            if len(tr_elements) != 3:
                self.get_logger().fatal(f"Failed to build transitions table: invalid transition format: {tr}")
                ok = False
                break
            tran_table.append(
                [
                    tr_elements[0].lower(),  # Trigger
                    tr_elements[1].upper(),  # Source state (may be a regex)
                    tr_elements[2].upper()   # Destination state
                ]
            )
        if not ok:
            if self._routines_module_name in sys.modules:
                del sys.modules[self._routines_module_name]
                self._routines_module = None
                gc.collect()
            self._routines_module_name = ""
            return False

        # Build states table
        state_table = []
        state_array: list[str] = self.get_parameter(
            "transitions.states").get_parameter_value().string_array_value
        for state in state_array:
            state_name = state.upper()
            state_routine_name = state_name.lower() + "_routine"
            state_routine = getattr(
                self._routines_module, state_routine_name, None)
            if state_routine is None or not callable(state_routine):
                self.get_logger().fatal(
                    f"State routine '{state_routine_name}' not found in module '{self._routines_module_name}'")
                ok = False
                break
            state_table.append({"name": state_name, "routine": state_routine})
        if not ok:
            if self._routines_module_name in sys.modules:
                del sys.modules[self._routines_module_name]
                self._routines_module = None
                gc.collect()
            self._routines_module_name = ""
            return False

        # Build state machine
        init_state = self.get_parameter(
            "transitions.initial_state").get_parameter_value().string_value
        try:
            self._fsm = Machine(
                node=self,
                states_table=state_table,
                transitions_table=tran_table,
                initial_state=init_state
            )
        except Exception as e:
            self.get_logger().fatal(f"Failed to create state machine: {e}")
            if self._routines_module_name in sys.modules:
                del sys.modules[self._routines_module_name]
                self._routines_module = None
                gc.collect()
            self._routines_module_name = ""
            return False

        # Start executor
        wait_servers = self.get_parameter("wait_servers").get_parameter_value().bool_value
        timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value
        self._executor = FSMExecutorThread(
            node=self,
            fsm=self._fsm,
            wait_servers=wait_servers,
            timeout_sec=timeout_sec
        )
        self._executor.start()

        return True

    def _stop_executor(self):
        """Stops FSM executor thread."""
        if self._executor is not None:
            if self._executor.is_alive():
                self._executor.stop()
            self._executor.join()
            self._executor = None

            # Clean up data structures
            if not self._blackboard_persistent:
                self._blackboard = {}
            self._fsm = None
            self.entities_map = {}

            # Unload routines module
            if self._routines_module_name in sys.modules:
                del sys.modules[self._routines_module_name]
                self._routines_module = None
                gc.collect()
                self.get_logger().info(
                    f"Unloaded routines module: {self._routines_module_name}")
            else:
                self.get_logger().error(
                    f"Routines module {self._routines_module_name} not found in sys.modules")
