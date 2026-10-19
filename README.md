# time_parametrization

ROS 2 action server that retimes a MoveIt trajectory with **Time-Optimal Trajectory Generation (TOTG)**.

You give it a path (a `moveit_msgs/MotionPlanResponse` whose trajectory has positions but no timing); it
computes time stamps, velocities and accelerations that respect the joint limits, and returns the same
response with the retimed trajectory.

Typical use: plan a joint path with a planner that does not produce timing (for example a `graph_core`
planner), retime it here, then execute it with MoveIt or `ros2_control`.

## Features

- MoveIt 2 TOTG (`trajectory_processing::TimeOptimalTrajectoryGeneration`).
- Per-group TOTG settings (`resample_dt`, `path_tolerance`, `min_angle_change`).
- Optional per-group **enforced joint limits** that override the limits of the robot model.
- Velocity and acceleration scaling factors per request.
- Joints without acceleration limits get a default limit of 5 × velocity limit.
- Works with ROS 2 Humble (MoveIt `.h` headers) and Jazzy (MoveIt `.hpp` headers).

## Repository layout

| Path | Content |
|---|---|
| `time_parametrization/src/time_parametrization_node.cpp` | the action server (`apply_time_parametrization_server`) |
| `time_parametrization/config/example_config.yaml` | example parameter file |
| `time_parametrization/launch/apply_time_parametrization_server.launch.py` | launch file |
| `docs/TUTORIAL.md` | step-by-step tutorial |

The action definition is in the separate package `time_parametrization_msgs`.

## Dependencies

- ROS 2 Humble or Jazzy
- MoveIt 2 (`moveit_core`, `moveit_ros_planning`, `moveit_msgs`)
- `rclcpp`, `rclcpp_action`, `yaml-cpp`
- `time_parametrization_msgs`

## Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/time_parametrization.git
# time_parametrization_msgs must be in the workspace too
cd ~/ros2_ws
colcon build --packages-up-to time_parametrization
source install/setup.bash
```

## Action interface

**Default name:** `apply_time_parametrization` (parameter `action_name`)
**Type:** `time_parametrization_msgs/action/ApplyTimeParametrization`

| Part | Field | Description |
|---|---|---|
| Goal | `moveit_msgs/MotionPlanResponse unparametrized_motion_plan_response` | trajectory to retime; only positions are needed |
| Goal | `float64 max_velocity_scaling_factor` | in (0, 1]; any other value (also 0) means 1.0 |
| Goal | `float64 max_acceleration_scaling_factor` | in (0, 1]; any other value (also 0) means 1.0 |
| Goal | `string cartesian_speed_limited_link`, `float64 max_cartesian_speed` | **not implemented**: ignored with a warning |
| Result | `moveit_msgs/MotionPlanResponse motion_plan_response` | copy of the input with retimed `trajectory`, `trajectory_start` and `error_code` |
| Feedback | `string status`, `float32 progress` | progress of the request |

The result `error_code` is:

- `SUCCESS` when the trajectory was retimed;
- `INVALID_MOTION_PLAN` when the trajectory is empty;
- `INVALID_GROUP_NAME` when no planning group can be selected;
- `FAILURE` when TOTG fails.

### How the start state is chosen

If `trajectory_start` of the input response is filled, it is used as the start state. Otherwise the
first trajectory point is used (single-DOF joints only). The first trajectory point should be the current
robot configuration, otherwise MoveIt will refuse to execute the trajectory.

### How the planning group is chosen

1. If the `default_group_name` parameter is set, that group is used.
2. Otherwise, if only one group is configured, that group is used.
3. Otherwise, the server uses the only configured group that contains all the joints of the trajectory.
   If none or more than one group matches, the goal is rejected.

Note: the `group_name` field of the input response is **not** used.

## Node parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `action_name` | string | `apply_time_parametrization` | action name |
| `robot_description_param` | string | `robot_description` | parameter that contains the URDF |
| `group_configs_yaml` | string | `"[]"` | YAML sequence with the group configurations (see below) |
| `default_group_name` | string | `""` | if set, always use this group |

ROS 2 parameters cannot be a list of maps, so the group configuration is a **YAML string**:

```yaml
apply_time_parametrization_server:
  ros__parameters:
    action_name: apply_time_parametrization
    robot_description_param: robot_description
    group_configs_yaml: |
      - group_name: "manipulator"
        resample_dt: 0.001        # [s], default 0.1
        path_tolerance: 0.001     # default 0.1
        min_angle_change: 0.001   # default 0.001
        enforced_joint_limits:    # optional
          - joint_name: "shoulder_pan_joint"
            has_velocity_limits: true
            max_velocity: 1.0
            has_acceleration_limits: true
            max_acceleration: 2.0
```

Groups that are not in the robot model are skipped with a warning.

Supported keys of a joint limit entry (`moveit_msgs/JointLimits`): `joint_name` (required),
`has_position_limits`, `min_position`, `max_position`, `has_velocity_limits`, `max_velocity`,
`has_acceleration_limits`, `max_acceleration`, `has_jerk_limits`, `max_jerk`.

## Launch

```bash
ros2 launch time_parametrization apply_time_parametrization_server.launch.py \
  config_pkg:=time_parametrization \
  config_file:=config/example_config.yaml
```

- `config_pkg`: package whose `share/` folder contains the parameter file;
- `config_file`: path of the parameter file, relative to that folder.

The node must also receive the robot description (by default the `robot_description` parameter), for
example by launching it next to `move_group` with the same parameters.

## Using the action

See [docs/TUTORIAL.md](docs/TUTORIAL.md) for a complete Python client.
`surface_path_planning/scripts/send_raster_to_joint_path.py` is another example of a client.

## Troubleshooting

- **Goal rejected**: the trajectory is empty, or no group matches its joints. Check the server log and the
  `group_configs_yaml` / `default_group_name` parameters.
- **TOTG failed**: often caused by inconsistent limits, a scaling factor that is too small, or duplicated
  consecutive points. Try without enforced limits, with scaling factors 1.0, or with a larger `path_tolerance`.
- **Very slow trajectory**: the scaling factors are applied on top of the joint limits; check that the robot
  model has realistic velocity/acceleration limits.

## API documentation

The API reference of the classes and functions is in [docs/API.md](docs/API.md) (generated from the
Doxygen comments).

To browse it as HTML, run Doxygen from the repository folder:

```bash
doxygen Doxyfile
xdg-open doxygen/html/index.html
```

## License

BSD 3-Clause, see [LICENSE](LICENSE).
