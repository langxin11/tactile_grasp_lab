# robotiq_2f85_driver

Default ROS 2 Python execution layer for Robotiq 2F-85 with a dry-run-first default.

## Scope

- Exposes a `GripperCommand` action, a legacy position topic, and simple Trigger services.
- Wraps the Modbus logic from the repository root `main.py`.
- Keeps hardware access isolated from higher-level grasp logic.
- Is used by the default tactile grasp bringup path.

## Topics and services

- Subscribes: `/robotiq/command/position` as `std_msgs/msg/Int32`
- Publishes: `/robotiq/command/echo` as `std_msgs/msg/Int32`
- Publishes: `/robotiq/driver/status` as `sensor_interfaces/msg/GripperStatus`
- Action: `/robotiq_gripper_controller/gripper_cmd` as `control_msgs/action/GripperCommand`
- Services: `/robotiq/activate`, `/robotiq/open`, `/robotiq/close`, `/robotiq/stop`, `/robotiq/reconnect`

## Safety

- Default `dry_run` is `true`.
- Real hardware mode requires `pymodbus` and a reachable serial device.
- This package publishes command echo, not measured finger position.
