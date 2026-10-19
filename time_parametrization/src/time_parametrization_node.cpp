/**
 * @file time_parametrization_node.cpp
 * @brief ROS 2 action server that retimes a MoveIt trajectory with Time-Optimal Trajectory Generation (TOTG).
 *
 * The node offers the action @c time_parametrization_msgs/action/ApplyTimeParametrization:
 * it receives an unparametrized @c moveit_msgs/MotionPlanResponse, computes the time stamps,
 * velocities and accelerations with MoveIt TOTG, and returns the retimed response.
 */
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "moveit_msgs/msg/joint_limits.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/motion_plan_response.hpp"

// If Jazzy or newer, load .hpp. If humble or older, load .h
#if __has_include(<moveit/robot_model_loader/robot_model_loader.hpp>)
  #include "moveit/robot_model_loader/robot_model_loader.hpp"
  #include "moveit/robot_state/conversions.hpp"
  #include "moveit/robot_trajectory/robot_trajectory.hpp"
  #include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
  #define MOVEIT_ROBOT_MODEL_LOADER_HPP_AVAILABLE
#elif __has_include(<moveit/robot_model_loader/robot_model_loader.h>)
  #include "moveit/robot_model_loader/robot_model_loader.h"
  #include "moveit/robot_state/conversions.h"
  #include "moveit/robot_trajectory/robot_trajectory.h"
  #include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#else
  #error "MoveIt robot_model_loader header not found (.hpp nor .h). Check your MoveIt install."
#endif

#include <yaml-cpp/yaml.h>

#include "time_parametrization_msgs/action/apply_time_parametrization.hpp"

namespace
{
/**
 * @brief Clamp a velocity/acceleration scaling factor to its valid range.
 * @param v Requested factor.
 * @return @p v if it is in (0, 1], otherwise 1.0 (no scaling).
 */
double sanitizeScalingFactor(double v)
{
  // Allowed values are in (0,1]. Treat anything else as "unspecified" => 1.0.
  if (!(v > 0.0 && v <= 1.0))
    return 1.0;
  return v;
}

/**
 * @brief Build a MoveIt error code message.
 * @param code One of the moveit_msgs::msg::MoveItErrorCodes constants.
 * @return The message.
 */
moveit_msgs::msg::MoveItErrorCodes makeError(int32_t code)
{
  moveit_msgs::msg::MoveItErrorCodes e;
  e.val = code;
  return e;
}

// Minimal YAML -> JointLimits parser
/**
 * @brief Parse a joint limit entry of the @c enforced_joint_limits list.
 *
 * Supported keys: @c joint_name (required), @c has_position_limits, @c min_position, @c max_position,
 * @c has_velocity_limits, @c max_velocity, @c has_acceleration_limits, @c max_acceleration,
 * @c has_jerk_limits, @c max_jerk. Missing flags default to false and missing values to 0.
 * @param n YAML map.
 * @return The joint limits.
 * @throws std::runtime_error if @c joint_name is missing.
 */
moveit_msgs::msg::JointLimits parseJointLimits(const YAML::Node &n)
{
  moveit_msgs::msg::JointLimits jl;

  if (!n["joint_name"])
    throw std::runtime_error("JointLimits entry missing required key 'joint_name'");
  jl.joint_name = n["joint_name"].as<std::string>();

  auto getBool = [&](const char *k, bool def) -> bool { return n[k] ? n[k].as<bool>() : def; };
  auto getDouble = [&](const char *k, double def) -> double { return n[k] ? n[k].as<double>() : def; };

  jl.has_position_limits = getBool("has_position_limits", false);
  jl.min_position = getDouble("min_position", 0.0);
  jl.max_position = getDouble("max_position", 0.0);

  jl.has_velocity_limits = getBool("has_velocity_limits", false);
  jl.max_velocity = getDouble("max_velocity", 0.0);

  jl.has_acceleration_limits = getBool("has_acceleration_limits", false);
  jl.max_acceleration = getDouble("max_acceleration", 0.0);

  jl.has_jerk_limits = getBool("has_jerk_limits", false);
  jl.max_jerk = getDouble("max_jerk", 0.0);

  return jl;
}

} // namespace

/**
 * @brief Node offering the ApplyTimeParametrization action.
 *
 * Action goal contains:
 *   - moveit_msgs/MotionPlanResponse unparametrized_motion_plan_response
 *   - scaling factors
 *   - optional cartesian speed limiting fields (NOT implemented here)
 *
 * The server retimes the input trajectory with MoveIt TOTG and returns a new MotionPlanResponse.
 * Usage: create the node with std::make_shared, call init(), then spin it with a multi-threaded executor.
 */
class ApplyTimeParametrizationServer : public rclcpp::Node
{
public:
  using ActionT = time_parametrization_msgs::action::ApplyTimeParametrization;
  using GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;

  /**
   * @brief Constructor. Declares the node parameters; init() must be called afterwards.
   *
   * Parameters:
   * - @c action_name (string, default "apply_time_parametrization"): name of the action;
   * - @c robot_description_param (string, default "robot_description"): parameter holding the URDF;
   * - @c group_configs_yaml (string, default "[]"): YAML sequence with one entry per planning group
   *   (see loadGroupConfigs());
   * - @c default_group_name (string, default ""): if set, this group is always used.
   * @param options Node options.
   */
  ApplyTimeParametrizationServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : rclcpp::Node("apply_time_parametrization_server", options)
  {
    declare_parameter<std::string>("action_name", "apply_time_parametrization");
    declare_parameter<std::string>("robot_description_param", "robot_description");

    // Inline YAML sequence string (because ROS2 params can't be list-of-maps)
    declare_parameter<std::string>("group_configs_yaml", "[]");

    // Optional: if set non-empty, the node uses this group (skips inference).
    declare_parameter<std::string>("default_group_name", "");
  }

  /**
   * @brief Load the robot model and the group configurations, then start the action server.
   *
   * It must be called after the node is owned by a shared pointer (it uses shared_from_this()).
   * Joints without acceleration limits get a default limit of 5 times the velocity limit.
   * @return True on success.
   * @throws std::runtime_error if the robot model or the group configuration cannot be loaded.
   */
  bool init()
  {
    RCLCPP_INFO(get_logger(), "Initializing ApplyTimeParametrizationServer");

    // Load RobotModel
    const auto robot_description_param = get_parameter("robot_description_param").as_string();
    RCLCPP_INFO(get_logger(), "Loading RobotModel from parameter '%s'", robot_description_param.c_str());

    robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), robot_description_param);

    if (!robot_model_loader_)
      throw std::runtime_error("Failed to create RobotModelLoader");

    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_)
      throw std::runtime_error("Failed to load RobotModel from parameter '" + robot_description_param + "'");

    RCLCPP_INFO(get_logger(), "Loaded RobotModel '%s'", robot_model_->getName().c_str());

    // Ensure accelerations exist; set defaults if missing (5 * velocity)
    for (auto &joint_model : robot_model_->getJointModels())
    {
      const auto &joint_name = joint_model->getName();

      if (joint_model->getType() == moveit::core::JointModel::FIXED)
        continue;

      const moveit::core::VariableBounds &var_bounds = joint_model->getVariableBounds(joint_name);
      moveit::core::VariableBounds mutable_bounds = var_bounds;

      if (!mutable_bounds.acceleration_bounded_)
      {
        RCLCPP_WARN(get_logger(),
                    "Joint '%s' has no acceleration limits; adding default (5*velocity)",
                    joint_name.c_str());
        if (mutable_bounds.velocity_bounded_)
        {
          mutable_bounds.max_acceleration_ = 5.0 * mutable_bounds.max_velocity_;
          mutable_bounds.min_acceleration_ = -5.0 * mutable_bounds.max_velocity_;
          mutable_bounds.acceleration_bounded_ = true;
          joint_model->setVariableBounds(joint_name, mutable_bounds);
        }
        else
        {
          RCLCPP_WARN(get_logger(),
                      "Joint '%s' has no velocity limits; cannot set default acceleration limits",
                      joint_name.c_str());
        }
      }
    }

    loadGroupConfigs();

    // Action server
    const auto action_name = get_parameter("action_name").as_string();
    RCLCPP_INFO(get_logger(), "Creating action server on '%s'", action_name.c_str());

    action_server_ = rclcpp_action::create_server<ActionT>(
      this,
      action_name,
      std::bind(&ApplyTimeParametrizationServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ApplyTimeParametrizationServer::handleCancel, this, std::placeholders::_1),
      std::bind(&ApplyTimeParametrizationServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ApplyTimeParametrization action server ready on '%s'", action_name.c_str());
    return true;
  }

private:
  /**
   * @brief TOTG configuration of one planning group.
   */
  struct GroupEntry
  {
    std::string group_name;  ///< MoveIt planning group name.
    std::shared_ptr<trajectory_processing::TimeOptimalTrajectoryGeneration> totg;  ///< TOTG instance with the group settings.

    bool has_enforced_joint_limits{false};  ///< True if enforced_joint_limits overrides the robot model limits.
    std::vector<moveit_msgs::msg::JointLimits> enforced_joint_limits;  ///< Joint limits that override the robot model ones.
  };

  /**
   * @brief Parse the @c group_configs_yaml parameter.
   *
   * Each entry of the YAML sequence supports:
   * - @c group_name (required): MoveIt planning group; entries of unknown groups are skipped;
   * - @c path_tolerance (default 0.1): TOTG path tolerance;
   * - @c resample_dt (default 0.1 s): TOTG resampling period;
   * - @c min_angle_change (default 0.001): TOTG minimum angle change;
   * - @c enforced_joint_limits (optional): list of joint limits, see parseJointLimits().
   * @throws std::runtime_error if the YAML is not a sequence or an entry is malformed.
   */
  void loadGroupConfigs()
  {
    std::lock_guard<std::mutex> lk(groups_mutex_);
    groups_.clear();

    const std::string yaml_text = get_parameter("group_configs_yaml").as_string();
    RCLCPP_INFO(get_logger(), "Parsing group configs YAML:\n%s", yaml_text.c_str());

    YAML::Node root = YAML::Load(yaml_text);
    if (!root || !root.IsSequence())
      throw std::runtime_error("'group_configs_yaml' must be a YAML sequence string (starts with '-')");

    if (root.size() == 0)
      RCLCPP_WARN(get_logger(), "No group configs found in 'group_configs_yaml' parameter");

    for (const auto &g : root)
    {
      if (!g["group_name"])
        throw std::runtime_error("Group entry missing required key 'group_name'");

      GroupEntry entry;
      entry.group_name = g["group_name"].as<std::string>();
      RCLCPP_INFO(get_logger(), "Loading config for group '%s'", entry.group_name.c_str());

      if (!robot_model_->hasJointModelGroup(entry.group_name))
      {
        RCLCPP_WARN(get_logger(), "Group '%s' not found in RobotModel; skipping", entry.group_name.c_str());
        continue;
      }

      const double path_tolerance   = g["path_tolerance"]   ? g["path_tolerance"].as<double>()   : 0.1;
      const double resample_dt      = g["resample_dt"]      ? g["resample_dt"].as<double>()      : 0.1;
      const double min_angle_change = g["min_angle_change"] ? g["min_angle_change"].as<double>() : 0.001;

      entry.totg = std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>(
        path_tolerance, resample_dt, min_angle_change);

      if (g["enforced_joint_limits"])
      {
        const auto &jl = g["enforced_joint_limits"];
        if (!jl.IsSequence())
          throw std::runtime_error("'enforced_joint_limits' must be a sequence for group '" + entry.group_name + "'");

        entry.has_enforced_joint_limits = true;
        entry.enforced_joint_limits.reserve(jl.size());
        for (const auto &jln : jl)
          entry.enforced_joint_limits.push_back(parseJointLimits(jln));
      }

      groups_.emplace(entry.group_name, std::move(entry));
    }

    if (groups_.empty())
      RCLCPP_WARN(get_logger(), "No valid group configs loaded; all goals will be rejected.");
  }

  // Build start state:
  // - prefer goal.unparametrized_motion_plan_response.trajectory_start if populated
  // - otherwise infer from first point of joint_trajectory
  /**
   * @brief Build the start robot state of the trajectory.
   *
   * Uses @c trajectory_start of the response if it is set, otherwise the first point of the joint
   * trajectory (single-DOF joints only). Joints not in the trajectory keep their default value.
   * @param resp Input motion plan response.
   * @param traj_msg Input trajectory.
   * @return Start state.
   */
  moveit::core::RobotState makeStartStateFromResponse(
    const moveit_msgs::msg::MotionPlanResponse& resp,
    const moveit_msgs::msg::RobotTrajectory& traj_msg) const
  {
    moveit::core::RobotState st(robot_model_);
    st.setToDefaultValues();

    if (!resp.trajectory_start.joint_state.name.empty())
    {
      moveit::core::robotStateMsgToRobotState(resp.trajectory_start, st);
      return st;
    }

    const auto& jt = traj_msg.joint_trajectory;
    if (jt.joint_names.empty() || jt.points.empty())
    {
      RCLCPP_WARN(get_logger(), "No trajectory_start and empty joint trajectory; using default RobotState");
      return st;
    }

    const auto& p0 = jt.points.front();
    if (p0.positions.size() != jt.joint_names.size())
    {
      RCLCPP_WARN(get_logger(),
                  "First point positions size (%zu) != joint_names size (%zu); using default RobotState",
                  p0.positions.size(), jt.joint_names.size());
      return st;
    }

    for (size_t i = 0; i < jt.joint_names.size(); ++i)
    {
      const std::string& joint_name = jt.joint_names[i];

      const auto* jm = robot_model_->getJointModel(joint_name);
      if (!jm)
        continue;

      const auto& vars = jm->getVariableNames();
      if (vars.size() != 1)
        continue; // ignore multi-DOF joints in this fallback

      st.setVariablePosition(vars[0], p0.positions[i]);
    }

    st.update();
    return st;
  }

  // Decide which MoveIt group to use:
  // - if default_group_name param is non-empty => use it
  // - else, if only one configured group => use it
  // - else, pick the unique group whose variable names contain all trajectory joint_names
  /**
   * @brief Select the planning group used to retime a trajectory.
   * @param traj_joint_names Joint names of the trajectory.
   * @return The @c default_group_name parameter if set; otherwise the only configured group;
   * otherwise the unique configured group containing all the trajectory joints.
   * @throws std::runtime_error if no group or more than one group matches.
   */
  std::string chooseGroupName(const std::vector<std::string>& traj_joint_names) const
  {
    const std::string forced = get_parameter("default_group_name").as_string();
    if (!forced.empty())
      return forced;

    if (groups_.empty())
      throw std::runtime_error("No group configs loaded.");

    if (groups_.size() == 1)
      return groups_.begin()->first;

    std::vector<std::string> matches;
    matches.reserve(groups_.size());

    for (const auto& kv : groups_)
    {
      const std::string& group_name = kv.first;
      const auto* jmg = robot_model_->getJointModelGroup(group_name);
      if (!jmg)
        continue;

      const auto& vars = jmg->getVariableNames();

      bool ok = true;
      for (const auto& jn : traj_joint_names)
      {
        if (std::find(vars.begin(), vars.end(), jn) == vars.end())
        {
          ok = false;
          break;
        }
      }
      if (ok)
        matches.push_back(group_name);
    }

    if (matches.size() == 1)
      return matches.front();

    if (matches.empty())
      throw std::runtime_error("Cannot infer group: no configured group matches trajectory joint names.");
    throw std::runtime_error("Cannot infer group: multiple configured groups match trajectory joint names. "
                             "Set parameter 'default_group_name'.");
  }

  /**
   * @brief Accept a goal if the trajectory is not empty and a group can be selected.
   * @return ACCEPT_AND_EXECUTE or REJECT.
   */
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const ActionT::Goal> goal)
  {
    const auto& resp = goal->unparametrized_motion_plan_response;
    const auto& jt   = resp.trajectory.joint_trajectory;

    if (jt.joint_names.empty() || jt.points.empty())
    {
      RCLCPP_WARN(get_logger(), "Rejecting goal: empty input joint_trajectory (joint_names or points).");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lk(groups_mutex_);
    if (groups_.empty())
    {
      RCLCPP_WARN(get_logger(), "Rejecting goal: no group configs loaded.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Optional: early group selection check (reject fast if ambiguous)
    try
    {
      (void)chooseGroupName(jt.joint_names);
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(get_logger(), "Rejecting goal: %s", e.what());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Cancellation is always accepted; it is checked between the execution steps.
   */
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleT> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Run execute() in a detached thread.
   */
  void handleAccepted(const std::shared_ptr<GoalHandleT> goal_handle)
  {
    std::thread{std::bind(&ApplyTimeParametrizationServer::execute, this, goal_handle)}.detach();
  }

  /**
   * @brief Retime the trajectory of the goal.
   *
   * The result is a copy of the input response where @c trajectory, @c trajectory_start and
   * @c error_code are replaced. Cartesian speed limits in the goal are ignored (a warning is printed).
   * Error codes: INVALID_MOTION_PLAN (empty trajectory), INVALID_GROUP_NAME (no group selected),
   * FAILURE (TOTG failed), SUCCESS.
   * @param goal_handle Goal handle.
   */
  void execute(const std::shared_ptr<GoalHandleT> goal_handle)
  {
    auto feedback = std::make_shared<ActionT::Feedback>();
    auto result   = std::make_shared<ActionT::Result>();

    const auto goal = goal_handle->get_goal();
    const auto& in_resp = goal->unparametrized_motion_plan_response;

    // Start by copying input response to output (then overwrite trajectory + error_code)
    result->motion_plan_response = in_resp;

    feedback->status = "Goal received; validating input";
    feedback->progress = 0.05f;
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      return;
    }

    const auto& in_traj = in_resp.trajectory;
    const auto& jt = in_traj.joint_trajectory;

    if (jt.joint_names.empty() || jt.points.empty())
    {
      RCLCPP_ERROR(get_logger(), "Input MotionPlanResponse has empty joint_trajectory.");
      result->motion_plan_response.error_code =
        makeError(moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
      goal_handle->abort(result);
      return;
    }

    // Note: goal includes cartesian speed limiting fields, but that is typically handled
    // by MoveIt's planning request adapter LimitMaxCartesianLinkSpeed.
    // This server only retimes joint trajectories via TOTG.
    if (goal->max_cartesian_speed > 0.0)
    {
      RCLCPP_WARN(get_logger(),
                  "max_cartesian_speed>0 requested (link='%s', v=%.3f m/s) but this server does not implement "
                  "cartesian speed limiting; ignoring.",
                  goal->cartesian_speed_limited_link.c_str(),
                  goal->max_cartesian_speed);
    }

    GroupEntry group_entry;
    std::string group_name;
    {
      std::lock_guard<std::mutex> lk(groups_mutex_);
      try
      {
        group_name = chooseGroupName(jt.joint_names);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(get_logger(), "Failed to choose group: %s", e.what());
        result->motion_plan_response.error_code =
          makeError(moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME);
        goal_handle->abort(result);
        return;
      }

      auto it = groups_.find(group_name);
      if (it == groups_.end())
      {
        RCLCPP_ERROR(get_logger(), "Internal error: chosen group '%s' not found in group map", group_name.c_str());
        result->motion_plan_response.error_code =
          makeError(moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME);
        goal_handle->abort(result);
        return;
      }
      group_entry = it->second;
    }

    feedback->status = "Applying TOTG time parametrization";
    feedback->progress = 0.2f;
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      return;
    }

    try
    {
      moveit::core::RobotState start_state = makeStartStateFromResponse(in_resp, in_traj);

      robot_trajectory::RobotTrajectory robot_traj(robot_model_, group_name);
      robot_traj.setRobotTrajectoryMsg(start_state, in_traj);

      const double vel_scale = sanitizeScalingFactor(goal->max_velocity_scaling_factor);
      const double acc_scale = sanitizeScalingFactor(goal->max_acceleration_scaling_factor);

      bool ok = false;
      if (group_entry.has_enforced_joint_limits)
      {
#ifdef MOVEIT_ROBOT_MODEL_LOADER_HPP_AVAILABLE
        ok = group_entry.totg->computeTimeStamps(robot_traj,
                                                 group_entry.enforced_joint_limits,
                                                 vel_scale,
                                                 acc_scale);
#else
        ok = group_entry.totg->computeTimeStamps(robot_traj, vel_scale, acc_scale);
#endif
      }
      else
      {
        ok = group_entry.totg->computeTimeStamps(robot_traj, vel_scale, acc_scale);
      }

      if (!ok)
      {
        RCLCPP_ERROR(get_logger(), "TOTG computeTimeStamps() failed (group='%s')", group_name.c_str());
        result->motion_plan_response.error_code =
          makeError(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
        goal_handle->abort(result);
        return;
      }

      moveit_msgs::msg::RobotTrajectory timed_msg;
      robot_traj.getRobotTrajectoryMsg(timed_msg);
      result->motion_plan_response.trajectory = timed_msg;

      // Keep trajectory_start consistent with the state used
      moveit::core::robotStateToRobotStateMsg(start_state, result->motion_plan_response.trajectory_start);

      result->motion_plan_response.error_code =
        makeError(moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Exception during TOTG: %s", ex.what());
      result->motion_plan_response.error_code =
        makeError(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
      goal_handle->abort(result);
      return;
    }

    feedback->status = "Time parametrization complete";
    feedback->progress = 1.0f;
    goal_handle->publish_feedback(feedback);

    goal_handle->succeed(result);
  }

private:
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;  ///< Robot model loader.
  moveit::core::RobotModelPtr robot_model_;  ///< Robot model.

  rclcpp_action::Server<ActionT>::SharedPtr action_server_;  ///< Action server.

  mutable std::mutex groups_mutex_;  ///< Protects groups_.
  std::unordered_map<std::string, GroupEntry> groups_;  ///< Group configurations, by group name.
};

/**
 * @brief Start the node with a multi-threaded executor.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<ApplyTimeParametrizationServer>();

  if (!node->init())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize ApplyTimeParametrizationServer");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "starting the server");
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

