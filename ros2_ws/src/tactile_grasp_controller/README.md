# tactile_grasp_controller

ROS 2 tactile grasp controller package for the Robotiq 2F-85 stack.

## Scope

- Subscribes to left and right tactile topics.
- Computes basic contact, force, friction, and slip features.
- Runs a small rule-based state machine.
- Sends `control_msgs/action/GripperCommand` goals to the `robotiq_2f85_driver` execution layer.

## External dependency

This package is designed to work with `sensor_interfaces/msg/SensorState` from the Contactile workspace.

Until `sensor_interfaces` is present in the workspace or underlay:

- the node can still start,
- debug and state publishers still work,
- tactile subscriptions remain disabled,
- real tactile control cannot run.

## Default topics

- Left tactile: `/hub_0/sensor_0`
- Right tactile: `/hub_0/sensor_1`
- Gripper action: `/robotiq_gripper_controller/gripper_cmd`
- Legacy command fallback: `/robotiq/command/position`
- Legacy command echo: `/robotiq/command/echo`

## Bringup

Launch the full stack from this workspace:

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py
```

Useful overrides:

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py \
  com_port:=/dev/ttyACM0 \
  gripper_com_port:=/dev/ttyUSB0 \
  controller_config:=/path/to/tactile_grasp_controller.yaml
```

Disable one layer if needed:

```bash
ros2 launch tactile_grasp_controller tactile_grasp_bringup.launch.py start_gripper_execution:=false
```
