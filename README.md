# transitions_ros

Python library to implement a finite-state machine with ROS 2 capabilities, based on the [`transitions`](https://github.com/pytransitions/transitions) library.

## Contents

The [`transitions`](https://github.com/pytransitions/transitions) library implements FSMs with many different paradigms, and a vast API. This library, being aimed at the implementation of automata that run robots ranging from simple to fairly complex in nature, currently implements just a specific subset of those functionalities.

The library extends the `Machine` and `State` classes of the `transitions` library adding just a few new components, with the following goals in mind:

- **The focus is on the machine object.** `transitions` allows one to link the FSM to a particular object, so as to make the FSM track that object's state. This library, instead, focuses on the FSM itself, which is intended as a fully capable automata that can be used to control a robot, or a part of it, or a system, or a process, or anything else that can be modeled as a finite-state machine. The machine object is therefore not linked to any other object, and is not intended to be used as a state tracker.
- **The machine must be able to access a ROS network.** This is necessary to make the automata able to communicate with the rest of the system, and to be able to control the robot. The machine object is therefore initialized with a ROS node, and can access the ROS network through it.
- **The machine can either be *active* or *passive*.** A *passive* machine is similar to what `transitions` traditionally allows to implement: an object that passively tracks the state of something and that is updated externally by some other code of the application that is using it. This is still permitted, although it does not allow to implement an automata that can actively control a robot. An *active* machine, instead, can be launched and will progress autonomously until a *terminal state* is reached. This library allows such an implementation by assigning a `routine` to each state, that will be performed when a state is entered, and `run` and `run_step` methods that will start the machine and automatically make it progress, persistently or for a single iteration step, according to the transitions table, triggering them depending on the trigger that is returned by each state's routine.

## Usage

Refer to the [`transitions`](https://github.com/pytransitions/transitions) library for the basic usage of the FSMs.

To create a machine, either *active* or *passive*, all that is required is a ROS 2 node (if necessary), a state table, a transitions table, and the name of an initial state.

To define a transitions table, follow the `transitions` syntax:

```python
transitions_table = [
    ['START', 'INIT', 'WAIT_FOR'],
    ['READY', 'WAIT_FOR', 'FIRST_TAKEOFF'],
    ['AIRBORNE', 'FIRST_TAKEOFF', 'EXPLORE'],
    # ...
    ['MISSION_DONE', 'DONE', 'RTH'],
    ['AT_HOME', 'RTH', 'COMPLETED']
]
```

To create an *active* machine, one must also assign routines to each state in the table, that is functions defined elsewhere, like so:

```python
def init_routine(node: MachineExecutorNode, wait_servers: bool, timeout_sec: float) -> str:
    """
    Wait for the mission start command from the user.

    :param node: ROS 2 node to use.
    :returns: Next trigger.
    """
    node.get_logger().warn("SYSTEMS ONLINE")

# Similar definitions for other routines...

states_table = [
    {'name': 'INIT', 'routine': init_routine},
    {'name': 'RTH', 'routine': rth_routine},
    {'name': 'COMPLETED', 'routine': completed_routine},
    {'name': 'ERROR', 'routine': error_routine}
]
```

Note that the routines must accept a ROS 2 node as a parameter, so that they can access the ROS network.

All machines support the standard methods from the `transitions.Machine` class. In addition, *active* machines support the `run` method, that will start the machine and make it progress until a *terminal state* is reached.

### [`machine_executor`](transitions_ros/machine_executor_node.py)

For additional ease of use, a ROS 2 node designed to load and execute state machines is provided. This node loads the definition of an FSM and its routines from its parameters and a Python module, and can started and stopped independently and reconfigured at run time.

A [sample launch file](launch/machine_executor.launch.py) is provided as well.

---

## Copyright and License

Copyright 2025 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
