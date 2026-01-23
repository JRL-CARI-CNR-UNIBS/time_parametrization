# time_parametrization (ROS 2 Jazzy)

Action server that applies MoveIt 2 **Time-Optimal Trajectory Generation (TOTG)** time parametrization to a planned trajectory.

This package exposes an action, **`time_parametrization_msgs/action/ApplyTimeParametrization`**, which:
1. Accepts a `moveit_msgs/MotionPlanRequest` (the same request you would send to MoveIt for planning),
2. Calls a MoveIt planning service (default: `/plan_kinematic_path`),
3. Applies TOTG to the returned trajectory (optionally with enforced joint limits),
4. Returns the updated `moveit_msgs/MotionPlanResponse`.

## What you get

- Per-group TOTG configuration (different `resample_dt`, tolerances, etc. per planning group).
- Optional per-group **enforced joint limits** overriding the robot defaults.
- Simple integration point for “plan + retime” pipelines without modifying MoveIt internals.

## Interfaces

### Action

**Name (default):** `apply_time_parametrization`  
**Type:** `time_parametrization_msgs/action/ApplyTimeParametrization`

#### Goal
- `moveit_msgs/MotionPlanRequest motion_plan_request`

#### Result
- `moveit_msgs/MotionPlanResponse motion_plan_response`

#### Feedback
- `string status`
- `float32 progress`

## How it works

1. The server receives a `MotionPlanRequest`.
2. It checks that `motion_plan_request.group_name` is configured/managed.
3. It reads `motion_plan_request.max_velocity_scaling_factor` and `motion_plan_request.max_acceleration_scaling_factor`.
4. It calls the configured MoveIt planning service (default: `/plan_kinematic_path`, type `moveit_msgs/srv/GetMotionPlan`).
5. If planning succeeds, it builds a `robot_trajectory::RobotTrajectory` and runs:

- `trajectory_processing::TimeOptimalTrajectoryGeneration::computeTimeStamps(trajectory, vel_scale, acc_scale)`, or
- `...computeTimeStamps(trajectory, enforced_joint_limits, vel_scale, acc_scale)` if limits are provided.

6. The server returns the same `MotionPlanResponse`, but with the `trajectory` retimed.

## Dependencies

At minimum:
- ROS 2 Jazzy
- MoveIt 2 (`moveit_core`, `moveit_ros_planning`, `moveit_msgs`)
- `rclcpp`, `rclcpp_action`
- `yaml-cpp`
- `time_parametrization_msgs`

## Build

From your workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Configuration

The server loads a list of group entries from either:
- `group_configs_file` (absolute path to YAML), or
- `group_configs_yaml` (inline YAML string; mainly for quick tests)

Because ROS parameters do not natively support “list of maps”, the preferred approach is `group_configs_file` via the provided launch file.

### YAML schema

Top-level must be a YAML **sequence**. Each entry supports:

- `group_name` (string, required)
- `resample_dt` (double, optional; seconds)
- `path_tolerance` (double, optional)
- `min_angle_change` (double, optional)
- `enforced_joint_limits` (optional sequence of joint limit objects)

Example:

```yaml
- group_name: "arm1"
  resample_dt: 0.001
  path_tolerance: 0.001
  min_angle_change: 0.001
  enforced_joint_limits:
    - joint_name: "joint_1"
      has_velocity_limits: true
      max_velocity: 1.0
      has_acceleration_limits: true
      max_acceleration: 2.0
    - joint_name: "joint_2"
      has_velocity_limits: true
      max_velocity: 1.2
      has_acceleration_limits: true
      max_acceleration: 2.5

- group_name: "arm2"
  resample_dt: 0.002
  path_tolerance: 0.002
  min_angle_change: 0.001
```

#### Supported `moveit_msgs/JointLimits` fields (Jazzy)

The YAML joint limit objects may contain:

- `joint_name` (required)
- `has_position_limits`, `min_position`, `max_position`
- `has_velocity_limits`, `max_velocity`
- `has_acceleration_limits`, `max_acceleration`
- `has_jerk_limits`, `max_jerk`

Note: `angle_wraparound` is not present in `moveit_msgs/JointLimits` on Jazzy.

## Node parameters

Common parameters (names as used in the reference implementation):

- `action_name` (string, default: `apply_time_parametrization`)
- `planning_service` (string, default: `/plan_kinematic_path`)
- `planning_service_wait_sec` (double, default: `2.0`)
- `robot_description_param` (string, default: `robot_description`)
- `group_configs_file` (string, default: empty)
- `group_configs_yaml` (string, default: `"[]"`)

## Launch

A convenience launch file is provided that lets you specify the config by **package name** and **relative file path** inside that package’s share directory.

Example:

```bash
ros2 launch time_parametrization apply_time_parametrization.launch.py \
  config_pkg:=time_parametrization \
  config_file:=config/group_configs.yaml
```

- `config_pkg`: the package containing the YAML in its `share/<pkg>/...`
- `config_file`: path relative to the package share directory (e.g., `config/group_configs.yaml`)

## Using the action

This action is easiest to call from a client node that already constructs a `moveit_msgs/MotionPlanRequest`
(start state, goal constraints, etc.). The returned `MotionPlanResponse` contains the retimed `trajectory`.

If you want a CLI test with `ros2 action send_goal`, be aware that `MotionPlanRequest` is verbose; in practice,
a small Python/C++ client is more convenient.

## Troubleshooting

- **Goal rejected (group not managed):** Ensure your YAML contains an entry for `motion_plan_request.group_name`
  and the group exists in the robot model.
- **Planning service unavailable:** Ensure MoveIt’s planning service is running (typically from `move_group`) and
  the `planning_service` parameter matches the correct name.
- **TOTG failed:** Often caused by invalid limits/scaling factors, inconsistent joint limits, or pathological paths.
  Try removing enforced limits, setting scaling factors to 1.0, or increasing `path_tolerance`.

## Notes / design constraints

- The action goal contains only a `MotionPlanRequest`, not a trajectory. Therefore, the server must either:
   - perform planning itself, or
   - call a planning service and retime the result (this package uses that approach by default).

If you want a pure “retime this trajectory” API, consider extending the action goal to include
a `moveit_msgs/RobotTrajectory` or a full `MotionPlanResponse`.