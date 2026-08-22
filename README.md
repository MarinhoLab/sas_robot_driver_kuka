# sas_robot_driver_kuka

> [!TIP]
> More information about the SmartArmStack is available in https://smartarmstack.github.io/.

> [!IMPORTANT]
> Do not clone this repository directly. See https://github.com/MarinhoLab/sas_kuka_control_template

## ROS 2 Nodes & Parameters

### Node: `sas_robot_driver_kuka_node`

| Property | Value |
|---|---|
| **Executable** | `sas_robot_driver_kuka_node` |
| **ROS node name** | `sas_robot_driver_kuka` |
| **Description** | Main driver node for the KUKA robot. Reads all parameters, instantiates `RobotDriverKuka` (FRI interface to the robot, via the joint-command overlay client) and `RobotDriverROS` (runs the control loop). |

#### Parameters

The parameters are provided through a YAML config file passed to the launch file (see `config/config.yaml`):

```bash
ros2 launch sas_robot_driver_kuka robot_launch.py name:=kuka_1
# or, with an alternate config file:
ros2 launch sas_robot_driver_kuka robot_launch.py name:=kuka_1 config_file:=/path/to/config.yaml
```

| Parameter | Type | Mandatory / Optional | Default | Purpose |
|---|---|---|---|---|
| `joint_limits_min` | array of 7 doubles (degrees) | **Mandatory** | none — must be provided | Minimum joint limits, in degrees (e.g. `[-170.0, -120.0, -170.0, -120.0, -170.0, -120.0, -175.0]` for the R820) |
| `joint_limits_max` | array of 7 doubles (degrees) | **Mandatory** | none — must be provided | Maximum joint limits, in degrees (e.g. `[170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0]` for the R820) |
| `thread_sampling_time_sec` | double | **Mandatory** | none — must be provided | Sampling period of the robot control-loop thread (e.g. `0.001` s = 1000 Hz) |

**How mandatory/optional is determined in code:**
- **Mandatory** params are read with `sas::get_ros_parameter(...)` — if missing, the node throws and fails to start.
- **Optional** params would be read with `sas::get_ros_optional_parameter(..., <default>)` — they carry in-code defaults. This node currently has no optional parameters.
