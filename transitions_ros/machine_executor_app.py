"""
transitions_ros machine executor node standalone application.

August 28, 2025
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

import sys

import rclpy
from rclpy.executors import MultiThreadedExecutor

from transitions_ros.machine_executor_node import MachineExecutorNode


def main(args=None):
    rclpy.init(args=args)

    node = MachineExecutorNode("machine_executor_node", verbose=True)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except:
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()
