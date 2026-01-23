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
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "moveit_msgs/msg/motion_plan_response.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"

#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/robot_state/conversions.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"

#include <yaml-cpp/yaml.h>

#include "time_parametrization_msgs/action/apply_time_parametrization.hpp"

namespace
{
  /**
   * @brief Ensure provided scaling factor is in the expected range.
   *
   * MoveIt historically expects a scaling factor in (0,1]. Treat any value
   * <= 0 or > 1 as unspecified and return 1.0.
   *
   * @param v requested scaling factor
   * @return sanitized scaling factor in (0,1]
   */
  double sanitizeScalingFactor(double v)
  {
    // MoveIt expects [0,1]. Treat <=0 as "unspecified" -> 1.0.
    if (!(v > 0.0 && v <= 1.0))
      return 1.0;
    return v;
  }

  /**
   * @brief Helper to construct MoveIt error message container.
   * @param code numeric MoveIt error code
   * @return moveit_msgs::msg::MoveItErrorCodes with .val set
   */
  moveit_msgs::msg::MoveItErrorCodes makeError(int32_t code)
  {
    moveit_msgs::msg::MoveItErrorCodes e;
    e.val = code;
    return e;
  }

  /**
   * @brief Convert a YAML node into a string (emitter output).
   * @param n YAML node to stringify
   * @return serialized YAML text
   */
  std::string yamlNodeToString(const YAML::Node &n)
  {
    YAML::Emitter out;
    out << n;
    return std::string(out.c_str());
  }

  /**
   * @brief Parse a YAML node describing joint limits into the ROS message type.
   *
   * This is a minimal parser that expects at least the key `joint_name` and
   * optionally the usual JointLimits fields. Throws std::runtime_error on
   * malformed input.
   */
  moveit_msgs::msg::JointLimits parseJointLimits(const YAML::Node &n)
  {
    moveit_msgs::msg::JointLimits jl;

    if (!n["joint_name"])
      throw std::runtime_error("JointLimits entry missing required key 'joint_name'");
    jl.joint_name = n["joint_name"].as<std::string>();

    auto getBool = [&](const char *k, bool def) -> bool
    {
      return n[k] ? n[k].as<bool>() : def;
    };
    auto getDouble = [&](const char *k, double def) -> double
    {
      return n[k] ? n[k].as<double>() : def;
    };

    // Position limits
    jl.has_position_limits = getBool("has_position_limits", false);
    jl.min_position = getDouble("min_position", 0.0);
    jl.max_position = getDouble("max_position", 0.0);

    // Velocity limits
    jl.has_velocity_limits = getBool("has_velocity_limits", false);
    jl.max_velocity = getDouble("max_velocity", 0.0);

    // Acceleration limits
    jl.has_acceleration_limits = getBool("has_acceleration_limits", false);
    jl.max_acceleration = getDouble("max_acceleration", 0.0);

    // Jerk limits (present in Jazzy JointLimits.msg)
    jl.has_jerk_limits = getBool("has_jerk_limits", false);
    jl.max_jerk = getDouble("max_jerk", 0.0);

    return jl;
  }
} // namespace

/**
 * @brief ROS2 action server node that applies time parametrization (TOTG)
 *        to planned joint trajectories.
 *
 * The node accepts an ApplyTimeParametrization action goal which contains a
 * MotionPlanRequest. It forwards that request to a planning service (e.g.
 * MoveIt's /plan_kinematic_path), receives a RobotTrajectory and then
 * computes time stamps using TimeOptimalTrajectoryGeneration. Group-specific
 * configuration (resampling dt, tolerances, and optional enforced joint
 * limits) is provided via a YAML parameter.
 */
class ApplyTimeParametrizationServer : public rclcpp::Node
{
public:
  using ActionT = time_parametrization_msgs::action::ApplyTimeParametrization;
  using GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;

  /**
   * @brief Construct the node and declare parameters.
   * @param options NodeOptions forwarded to rclcpp::Node
   */
  ApplyTimeParametrizationServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : rclcpp::Node("apply_time_parametrization_server", options)
  {
    declare_parameter<std::string>("action_name", "apply_time_parametrization");
    declare_parameter<std::string>("planning_service", "/plan_kinematic_path");
    declare_parameter<double>("planning_service_wait_sec", 2.0);
    // Robot model loading
    declare_parameter<std::string>("robot_description_param", "robot_description");

    // Group config input: either a file or an inline YAML string.
    // Because ROS 2 parameters do not support "list of maps" types, we treat it as YAML text.
    declare_parameter<std::string>("group_configs_yaml", "[]");
  }

  bool init()
  {
    RCLCPP_INFO(get_logger(), "Initializing ApplyTimeParametrizationServer");


    RCLCPP_INFO(get_logger(), "Loading RobotModel...");
    const auto robot_description_param = get_parameter("robot_description_param").as_string();
    RCLCPP_INFO(get_logger(), "Using robot description parameter: %s", robot_description_param.c_str());
    robot_model_loader_ =
        std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), robot_description_param);
    if (!robot_model_loader_)
    {
      throw std::runtime_error("Failed to create RobotModelLoader with parameter '" + robot_description_param + "'");
    }
    RCLCPP_INFO(get_logger(), "RobotModelLoader created");
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_)
    {
      throw std::runtime_error("Failed to load RobotModel from parameter '" + robot_description_param + "'");
    }
    RCLCPP_INFO(get_logger(), "Loaded RobotModel '%s'", robot_model_->getName().c_str());

    // check if joints has accelerations, if not add default values (5*velocities)
    for (auto &joint_model : robot_model_->getJointModels())
    {
      const auto &joint_name = joint_model->getName();

      // skip if fixed joint
      if (joint_model->getType() == moveit::core::JointModel::FIXED)
        continue;
      // get const Variable bounds
      const moveit::core::VariableBounds &var_bounds = joint_model->getVariableBounds(joint_name);
      // copy to mutable bound
      moveit::core::VariableBounds mutable_var_bounds = var_bounds;
      if (!mutable_var_bounds.acceleration_bounded_)
      {
        RCLCPP_WARN(get_logger(),
                    "Joint '%s' has no acceleration limits defined; adding default limits (5 * velocity limits)",
                    joint_name.c_str());
        if (mutable_var_bounds.velocity_bounded_)
        {
          mutable_var_bounds.max_acceleration_ = 5.0 * mutable_var_bounds.max_velocity_;
          mutable_var_bounds.min_acceleration_ = -5.0 * mutable_var_bounds.max_velocity_;
          mutable_var_bounds.acceleration_bounded_ = true;
          joint_model->setVariableBounds(joint_name, mutable_var_bounds);
        }
        else
        {
          RCLCPP_WARN(get_logger(),
                      "Joint '%s' has no velocity limits defined; cannot set default acceleration limits",
                      joint_name.c_str());
        }
      }
    }

    loadGroupConfigs();

    // Planning service client (MoveIt move_group typically provides /plan_kinematic_path)
    const auto planning_service = get_parameter("planning_service").as_string();
    plan_client_ = create_client<moveit_msgs::srv::GetMotionPlan>(planning_service);

    // Action server
    const auto action_name = get_parameter("action_name").as_string();
    RCLCPP_INFO(get_logger(), "Creating ApplyTimeParametrization action server on '%s'", action_name.c_str());
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
  struct GroupEntry
  {
    std::string group_name;
    std::shared_ptr<trajectory_processing::TimeOptimalTrajectoryGeneration> totg;

    bool has_enforced_joint_limits{false};
    std::vector<moveit_msgs::msg::JointLimits> enforced_joint_limits;
  };

  void loadGroupConfigs()
  {
    std::lock_guard<std::mutex> lk(groups_mutex_);
    groups_.clear();

    const std::string yaml_text = get_parameter("group_configs_yaml").as_string();
    RCLCPP_INFO(get_logger(), "Parsing group configs YAML:\n%s", yaml_text.c_str());
    YAML::Node root = YAML::Load(yaml_text);

    if (!root || !root.IsSequence())
      throw std::runtime_error("'group_configs_yaml' must be a YAML sequence string (starts with '-')");

    const YAML::Node& group_list = root;

    RCLCPP_INFO(get_logger(), "Loading group configs from inline YAML parameter 'group_configs_yaml'");

    // const YAML::Node group_list = extractGroupConfigsNode(root, this->get_name());

    if (!group_list || !group_list.IsSequence())
    {
      // Emit a concise hint about what we expected
      throw std::runtime_error(
        "Group configs must be either:\n"
        "  - a top-level YAML sequence, OR\n"
        "  - a YAML map containing key 'group_configs_yaml' with a sequence, OR\n"
        "  - a ROS params YAML with '<node>: ros__parameters: group_configs_yaml: <sequence>'.");
    }
    if (group_list.size() == 0)
    {
      RCLCPP_WARN(get_logger(), "No group configs found in 'group_configs_yaml' parameter");
    }


    for (const auto &g: group_list)
    {
      if (!g["group_name"])
        throw std::runtime_error("Group entry missing required key 'group_name'");

      RCLCPP_INFO(get_logger(), "Loading config for group '%s'", g["group_name"].as<std::string>().c_str());

      GroupEntry entry;
      entry.group_name = g["group_name"].as<std::string>();

      if (!robot_model_->hasJointModelGroup(entry.group_name))
      {
        RCLCPP_WARN(get_logger(), "Configured group '%s' not found in RobotModel; skipping",
                    entry.group_name.c_str());
        continue;
      }


      const double path_tolerance = g["path_tolerance"] ? g["path_tolerance"].as<double>() : 0.1;
      const double resample_dt = g["resample_dt"] ? g["resample_dt"].as<double>() : 0.1;
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
        for (const auto &jln: jl)
        {
          entry.enforced_joint_limits.push_back(parseJointLimits(jln));
        }
      }

      groups_.emplace(entry.group_name, std::move(entry));
    }

    if (groups_.empty())
      RCLCPP_WARN(get_logger(), "No valid group configs loaded; all goals will be rejected.");
  }

  static YAML::Node extractGroupConfigsNode(const YAML::Node &root, const std::string &node_name)
  {
    // Case 1: root is already a sequence
    if (root && root.IsSequence())
      return root;

    if (!root || !root.IsMap())
      return YAML::Node();

    // Case 2: root has "group_configs_yaml: [ ... ]"
    if (root["group_configs_yaml"])
      return root["group_configs_yaml"];

    // Case 3: ROS params file style:
    // <node_name>:
    //   ros__parameters:
    //     group_configs_yaml: [ ... ]
    if (root[node_name] && root[node_name]["ros__parameters"] && root[node_name]["ros__parameters"][
          "group_configs_yaml"])
      return root[node_name]["ros__parameters"]["group_configs_yaml"];

    // Also support wildcard node key: "/**"
    if (root["/**"] && root["/**"]["ros__parameters"] && root["/**"]["ros__parameters"]["group_configs_yaml"])
      return root["/**"]["ros__parameters"]["group_configs_yaml"];

    return YAML::Node();
  }


  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &,
                                         std::shared_ptr<const ActionT::Goal> goal)
  {
    const std::string &group_name = goal->motion_plan_request.group_name;

    std::lock_guard<std::mutex> lk(groups_mutex_);
    if (groups_.find(group_name) == groups_.end())
    {
      RCLCPP_WARN(get_logger(), "Rejecting goal: group '%s' is not managed", group_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleT> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleT> goal_handle)
  {
    std::thread{std::bind(&ApplyTimeParametrizationServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleT> goal_handle)
  {
    auto feedback = std::make_shared<ActionT::Feedback>();
    auto result = std::make_shared<ActionT::Result>();

    const auto goal = goal_handle->get_goal();
    const std::string group_name = goal->motion_plan_request.group_name;

    GroupEntry group_entry;
    {
      std::lock_guard<std::mutex> lk(groups_mutex_);
      group_entry = groups_.at(group_name);
    }

    // Feedback: start
    feedback->status = "Planning request received";
    feedback->progress = 0.05f;
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      return;
    }

    // Wait for planning service
    const double wait_sec = get_parameter("planning_service_wait_sec").as_double();
    if (!plan_client_->wait_for_service(std::chrono::duration<double>(wait_sec)))
    {
      RCLCPP_ERROR(get_logger(), "Planning service not available within %.2f sec", wait_sec);
      result->motion_plan_response.error_code = makeError(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
      goal_handle->abort(result);
      return;
    }

    // Call planning service
    auto srv_req = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();
    srv_req->motion_plan_request = goal->motion_plan_request;

    feedback->status = "Calling planning service";
    feedback->progress = 0.15f;
    goal_handle->publish_feedback(feedback);

    auto future = plan_client_->async_send_request(srv_req);
    // Wait in this worker thread; executor spinning elsewhere will fulfill the future.
    if (future.wait_for(std::chrono::duration<double>(wait_sec)) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "Planning service call timed out after %.2f sec", wait_sec);
      result->motion_plan_response.error_code = makeError(moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT);
      goal_handle->abort(result);
      return;
    }

    auto srv_resp = future.get();
    result->motion_plan_response = srv_resp->motion_plan_response;

    if (result->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Planning failed with MoveItErrorCodes=%d",
                  result->motion_plan_response.error_code.val);
      goal_handle->abort(result);
      return;
    }

    feedback->status = "Planning succeeded; applying TOTG time parametrization";
    feedback->progress = 0.60f;
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      return;
    }

    // Build RobotTrajectory from the response and apply TOTG
    try
    {
      moveit::core::RobotState start_state(robot_model_);
      moveit::core::robotStateMsgToRobotState(result->motion_plan_response.trajectory_start, start_state);

      robot_trajectory::RobotTrajectory robot_traj(robot_model_, group_name);
      robot_traj.setRobotTrajectoryMsg(start_state, result->motion_plan_response.trajectory);

      const double vel_scale = sanitizeScalingFactor(goal->motion_plan_request.max_velocity_scaling_factor);
      const double acc_scale = sanitizeScalingFactor(goal->motion_plan_request.max_acceleration_scaling_factor);

      bool ok = false;
      if (group_entry.has_enforced_joint_limits)
      {
        ok = group_entry.totg->computeTimeStamps(robot_traj, group_entry.enforced_joint_limits, vel_scale, acc_scale);
      } else
      {
        ok = group_entry.totg->computeTimeStamps(robot_traj, vel_scale, acc_scale);
      }

      if (!ok)
      {
        RCLCPP_ERROR(get_logger(), "TOTG computeTimeStamps() failed for group '%s'", group_name.c_str());
        result->motion_plan_response.error_code = makeError(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
        goal_handle->abort(result);
        return;
      }

      moveit_msgs::msg::RobotTrajectory timed_msg;
      robot_traj.getRobotTrajectoryMsg(timed_msg);
      result->motion_plan_response.trajectory = timed_msg;
    } catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Exception during TOTG: %s", ex.what());
      result->motion_plan_response.error_code = makeError(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
      goal_handle->abort(result);
      return;
    }

    feedback->status = "Time parametrization complete";
    feedback->progress = 1.0f;
    goal_handle->publish_feedback(feedback);

    goal_handle->succeed(result);
  }

private:
  /**
   * @brief Safely get joint limits for a joint name from the local cache.
   *
   * This node keeps an internal override cache so it doesn't depend on the
   * RobotModel exposing get/set joint-limits methods. The cache is used when
   * enforced joint limits are provided via parameters or when the node sets
   * defaults.
   *
   * @param joint_name name of the joint
   * @return JointLimits message (default-constructed if not present)
   */
  moveit_msgs::msg::JointLimits getJointLimitsSafe(const std::string &joint_name) const
  {
    auto it = joint_limits_overrides_.find(joint_name);
    if (it != joint_limits_overrides_.end())
      return it->second;
    return moveit_msgs::msg::JointLimits();
  }

  /**
   * @brief Store joint limits into the local override cache.
   * @param joint_name joint to update
   * @param jl limits to store
   */
  void setJointLimitsSafe(const std::string &joint_name, const moveit_msgs::msg::JointLimits &jl)
  {
    joint_limits_overrides_[joint_name] = jl;
  }

private:
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;

  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr plan_client_;
  rclcpp_action::Server<ActionT>::SharedPtr action_server_;

  std::mutex groups_mutex_;
  std::unordered_map<std::string, GroupEntry> groups_;

  // Local cache for joint-limit overrides to avoid depending on RobotModel API
  // methods that might not be available in the installed MoveIt version.
  std::unordered_map<std::string, moveit_msgs::msg::JointLimits> joint_limits_overrides_;
};

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
