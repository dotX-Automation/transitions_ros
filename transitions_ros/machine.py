"""
Implementation of a ROS 2-compatible transitions machine.

Roberto Masocco <r.masocco@dotxautomation.com>

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

import transitions
from transitions_ros.state import *


class Machine(transitions.Machine):
    """Finite-state machine with ROS 2 capabilities."""

    # Use custom states
    state_cls = State

    def __init__(self,
                 node: NodeType,
                 states_table: list,
                 transitions_table: list,
                 initial_state: str) -> None:
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
        return State(node=self._node, *args, **kwargs)

    def run(self) -> None:
        """Runs the active FSM."""
        while True:
            # Execute the routine of the current state and get the next trigger
            next_trigger = self.get_model_state(self).routine()

            # If there's no trigger, we're done here
            if len(next_trigger) == 0:
                break

            # If the current state routine is non-blocking, keep doing this
            if next_trigger == 'RUNNING':
                continue

            # Get to the next state
            self.trigger(next_trigger)
