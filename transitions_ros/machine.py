"""
Implementation of a ROS 2-compatible transitions machine.

dotX Automation s.r.l. <info@dotxautomation.com>

September 11, 2022
"""

# Copyright 2024 dotX Automation s.r.l.
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

from enum import Enum

import transitions
from transitions_ros.state import *


class MachineStatus(Enum):
    IDLE = 0
    RUNNING = 1
    COMPLETED = 2


class Machine(transitions.Machine):
    """Finite-state machine with ROS 2 capabilities."""

    # Use custom states
    state_cls = State

    def __init__(self,
                 node: NodeType,
                 states_table: list,
                 transitions_table: list,
                 initial_state: str):
        """
        Creates a new Machine, linking a ROS 2 node to it.

        :param node: ROS 2 node to link.
        :param states_table: List of states to use.
        :param transitions_table: List of transitions to use.
        :param initial_state: Initial state of the machine.
        """
        # Set internal attributes
        self._node = node

        # Initialize transitions machine
        super().__init__(
            states=states_table,
            transitions=transitions_table,
            initial=initial_state)

    def _create_state(self, *args, **kwargs) -> State:
        """Creates a new State object, overriding the default method."""
        return State(*args, **kwargs)

    def run_step(self, wait_servers: bool, timeout_sec: float) -> MachineStatus:
        """
        Runs a step of the active FSM.

        :param wait_servers: Wait for servers to be available (flag passed to state routines).
        :param timeout_sec: Timeout for service/action client operations (passed to state routines).
        :returns: Current machine status.
        """
        # Execute the routine of the current state and get the next trigger
        next_trigger = self.get_model_state(self).routine(
            node=self._node,
            wait_servers=wait_servers,
            timeout_sec=timeout_sec
        )

        # If there's no trigger, we're done here
        if len(next_trigger) == 0:
            return MachineStatus.COMPLETED

        # If the current state routine is non-blocking, stay there
        if next_trigger == 'RUNNING':
            return MachineStatus.RUNNING

        # Get to the next state
        self.trigger(next_trigger)
        return MachineStatus.RUNNING

    def run(self, wait_servers: bool, timeout_sec: float) -> None:
        """
        Runs the active FSM, iterating/blocking until completion.

        :param wait_servers: Wait for servers to be available (flag passed to state routines).
        :param timeout_sec: Timeout for service/action client operations (passed to state routines).
        """
        curr_status = MachineStatus.IDLE
        while curr_status != MachineStatus.COMPLETED:
            curr_status = self.run_step(wait_servers, timeout_sec)
