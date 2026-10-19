# time_parametrization API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`time_parametrization/src/time_parametrization_node.cpp`](#time_parametrizationsrctime_parametrization_nodecpp)
  - [`ApplyTimeParametrizationServer`](#class-applytimeparametrizationserver)
  - [`ApplyTimeParametrizationServer::GroupEntry`](#struct-applytimeparametrizationservergroupentry)

## time_parametrization/src/time_parametrization_node.cpp

ROS 2 action server that retimes a MoveIt trajectory with Time-Optimal Trajectory Generation (TOTG).

The node offers the action `time_parametrization_msgs/action/ApplyTimeParametrization`: it receives an unparametrized `moveit_msgs/MotionPlanResponse`, computes the time stamps, velocities and accelerations with MoveIt TOTG, and returns the retimed response.

### class `ApplyTimeParametrizationServer`

Inherits from `rclcpp::Node`.

Node offering the ApplyTimeParametrization action.

Action goal contains:

- moveit_msgs/MotionPlanResponse unparametrized_motion_plan_response
- scaling factors
- optional cartesian speed limiting fields (NOT implemented here)

The server retimes the input trajectory with MoveIt TOTG and returns a new MotionPlanResponse. Usage: create the node with std::make_shared, call `init()`, then spin it with a multi-threaded executor.

#### Public types

##### `ActionT`

```cpp
using ApplyTimeParametrizationServer::ActionT = time_parametrization_msgs::action::ApplyTimeParametrization
```

##### `GoalHandleT`

```cpp
using ApplyTimeParametrizationServer::GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>
```

#### Public functions

##### `ApplyTimeParametrizationServer`

```cpp
ApplyTimeParametrizationServer(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
```

Constructor.

Declares the node parameters; `init()` must be called afterwards.

Parameters:

- `action_name` (string, default "apply_time_parametrization"): name of the action;
- `robot_description_param` (string, default "robot_description"): parameter holding the URDF;
- `group_configs_yaml` (string, default "[]"): YAML sequence with one entry per planning group (see loadGroupConfigs());
- `default_group_name` (string, default ""): if set, this group is always used.

**Parameters:**

- `options`: Node options.

##### `init`

```cpp
bool init()
```

Load the robot model and the group configurations, then start the action server.

It must be called after the node is owned by a shared pointer (it uses shared_from_this()). Joints without acceleration limits get a default limit of 5 times the velocity limit.

**Throws:**

- `std::runtime_error`: if the robot model or the group configuration cannot be loaded.

**Returns:** True on success.

### struct `ApplyTimeParametrizationServer::GroupEntry`

TOTG configuration of one planning group.

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `group_name` | `std::string` | MoveIt planning group name. |
| `totg` | `std::shared_ptr< trajectory_processing::TimeOptimalTrajectoryGeneration >` | TOTG instance with the group settings. |
| `has_enforced_joint_limits` | `bool` | True if enforced_joint_limits overrides the robot model limits. |
| `enforced_joint_limits` | `std::vector< moveit_msgs::msg::JointLimits >` | Joint limits that override the robot model ones. |

### Free functions and types

#### `main`

```cpp
int main(int argc, char **argv)
```

Start the node with a multi-threaded executor.
