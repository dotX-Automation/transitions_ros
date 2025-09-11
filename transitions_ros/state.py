"""
Implementation of the state of a ROS 2-compatible transitions machine.

dotX Automation s.r.l. <info@dotxautomation.com>

September 11, 2022
"""

# Copyright 2025 dotX Automation s.r.l.
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
from typing import TypeVar

NodeType = TypeVar('NodeType')
StateRoutineType = TypeVar('StateRoutineType')


class State(transitions.State):
    """Augmented machine state that also stores a ROS 2 node and a routine to execute."""

    def __init__(
            self,
            routine: StateRoutineType = None,
            *args,
            **kwargs):
        """
        Creates a new State object.

        :param node: ROS 2 node to use.
        :param routine: Routine associated with this state.
        """
        # Set internal data for this implementation
        self._routine = routine

        # Call State constructor
        super().__init__(*args, **kwargs)

    def routine(
        self,
        node: NodeType = None,
        wait_servers: bool = False,
        timeout_sec: float = 0.0
    ) -> str:
        """
        Executes the routine associated with this state and returns the next trigger.

        :param node: ROS 2 node to use.
        :param wait_servers: Wait for service/action servers to be available.
        :param timeout_sec: Timeout for service/action client operations.
        :returns: Next trigger from the routine.
        """
        return self._routine(
            node=node,
            wait_servers=wait_servers,
            timeout_sec=timeout_sec
        )
