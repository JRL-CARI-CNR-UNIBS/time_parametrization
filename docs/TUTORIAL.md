# time_parametrization tutorial

In this tutorial you start the time parametrization server for a robot and retime a simple joint path
from a Python client.

You need a robot with a MoveIt configuration (URDF + SRDF) and its `robot_description` parameter. The
examples use a UR robot with the planning group `ur_manipulator`; change joint and group names to match
your robot.

## 1. Write the configuration

Create `config/time_parametrization.yaml` in your own package (or copy `example_config.yaml`):

```yaml
apply_time_parametrization_server:
  ros__parameters:
    action_name: apply_time_parametrization
    robot_description_param: robot_description
    group_configs_yaml: |
      - group_name: "ur_manipulator"
        resample_dt: 0.008       # one point every 8 ms (125 Hz controller)
        path_tolerance: 0.01
        min_angle_change: 0.001
```

`resample_dt` is the time step of the output trajectory: use the period of your controller, or a larger
value if you only need a coarse trajectory.

If the robot model limits are too high for your application, add `enforced_joint_limits`:

```yaml
        enforced_joint_limits:
          - joint_name: "shoulder_pan_joint"
            has_velocity_limits: true
            max_velocity: 0.5
            has_acceleration_limits: true
            max_acceleration: 1.0
```

Install the `config` folder in `CMakeLists.txt` of your package:

```cmake
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
```

## 2. Start the server

The server reads the URDF from the `robot_description` parameter of its own node, so it needs the robot
description. The simplest way is to add it to your robot launch file, next to `move_group`:

```python
from launch_ros.actions import Node

time_parametrization_server = Node(
    package="time_parametrization",
    executable="apply_time_parametrization_server",
    name="apply_time_parametrization_server",   # must match the key in the YAML file
    output="screen",
    parameters=[
        moveit_config.robot_description,        # robot_description
        os.path.join(get_package_share_directory("my_robot_config"), "config", "time_parametrization.yaml"),
    ],
)
```

In the log you should see:

```
Loaded RobotModel '<robot name>'
Loading config for group 'ur_manipulator'
ApplyTimeParametrization action server ready on 'apply_time_parametrization'
```

Check the action:

```bash
ros2 action list -t | grep apply_time_parametrization
```

## 3. Write a client

`retime_example.py`:

```python
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.msg import MotionPlanResponse, MoveItErrorCodes
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from time_parametrization_msgs.action import ApplyTimeParametrization

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def main():
    rclpy.init()
    node = rclpy.create_node("retime_example")
    client = ActionClient(node, ApplyTimeParametrization, "apply_time_parametrization")
    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("server not available")
        return

    # A path with three waypoints and no timing. The first point must be the current robot configuration.
    waypoints = [
        [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        [0.5, -1.20, 1.20, -1.57, -1.57, 0.0],
        [1.0, -1.57, 1.57, -1.57, -1.57, 0.5],
    ]
    path = JointTrajectory(joint_names=JOINTS,
                           points=[JointTrajectoryPoint(positions=q) for q in waypoints])

    goal = ApplyTimeParametrization.Goal()
    goal.unparametrized_motion_plan_response = MotionPlanResponse()
    goal.unparametrized_motion_plan_response.trajectory.joint_trajectory = path
    goal.unparametrized_motion_plan_response.trajectory_start.joint_state = JointState(
        name=JOINTS, position=waypoints[0])
    goal.max_velocity_scaling_factor = 0.5      # 50 % of the maximum joint velocities
    goal.max_acceleration_scaling_factor = 0.5

    send_future = client.send_goal_async(
        goal, feedback_callback=lambda fb: node.get_logger().info(fb.feedback.status))
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if not handle.accepted:
        node.get_logger().error("goal rejected (see the server log)")
        return

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    response = result_future.result().result.motion_plan_response

    if response.error_code.val != MoveItErrorCodes.SUCCESS:
        node.get_logger().error(f"time parametrization failed, error code {response.error_code.val}")
        return

    traj = response.trajectory.joint_trajectory
    t_end = traj.points[-1].time_from_start
    node.get_logger().info(f"{len(traj.points)} points, duration {t_end.sec + t_end.nanosec * 1e-9:.2f} s")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Run it:

```bash
python3 retime_example.py
```

Example output:

```
[retime_example]: Goal received; validating input
[retime_example]: Applying TOTG time parametrization
[retime_example]: Time parametrization complete
[retime_example]: 312 points, 2.49 s
```

The exact numbers depend on the joint limits of your robot and on `resample_dt`.

## 4. Execute the trajectory

The result is a standard `moveit_msgs/RobotTrajectory`, so you can execute it with the MoveIt
`ExecuteTrajectory` action:

```python
from moveit_msgs.action import ExecuteTrajectory

exec_client = ActionClient(node, ExecuteTrajectory, "execute_trajectory")
exec_goal = ExecuteTrajectory.Goal(trajectory=response.trajectory)
exec_client.send_goal_async(exec_goal)
```

Or you can send `response.trajectory.joint_trajectory` directly to a `FollowJointTrajectory` controller.

## 5. Try the scaling factors

Run the client with different scaling factors and compare the durations:

| `max_velocity_scaling_factor` | `max_acceleration_scaling_factor` | Effect |
|---|---|---|
| 1.0 | 1.0 | fastest trajectory allowed by the limits |
| 0.5 | 0.5 | about twice as long |
| 0.1 | 0.1 | slow, good for first tests on a real robot |
| 0.0 | 0.0 | treated as 1.0 (fastest)! |

Values outside (0, 1], including 0, are replaced by 1.0. Always set the factors explicitly.

## 6. Several planning groups

With more than one group in `group_configs_yaml`, the server selects the group whose joints contain all
the trajectory joints. If two groups match (for example `arm` and `arm_with_gripper`), the goal is
rejected: set `default_group_name` to choose one explicitly.
