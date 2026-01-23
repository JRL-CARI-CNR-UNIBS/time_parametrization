#pragma once

#include <graph_core/graph/path.h>
#include <pose_constraints_planner/pose_constraints_manager.hpp>
// Class loader
#include <cnr_class_loader/multi_library_class_loader.hpp>

// IK solver
#include "ik_solver/internal/ik_solver_node.ros2.hpp"
#include <graph_core/graph/tree.h>


namespace pose_constraints_planner
{

  struct info
  {
    double total_time;
    int total_iterations;
    int total_rejections;
    double max_iteration_time;
    double min_iteration_time;
    double average_iteration_time;
    double total_reject_time;
  };

  class PoseConstraintsPlanner
  {
  public:
    PoseConstraintsPlanner(rclcpp::Node::SharedPtr node,
                           graph::core::CollisionCheckerPtr collision_checker,
                           std::shared_ptr<ik_solver::IkSolver> ik_solver,
                           graph::core::SamplerPtr sampler,
                           graph::core::MetricsPtr metrics,
                           cnr_logger::TraceLoggerPtr logger_ptr,
                           std::string world_frame = "world",
                           std::string tool_frame = "ur10_tool0");

    /* Set geometric constraints from a GeometricConstraintArray message
     * @param constraints The GeometricConstraintArray message containing the constraints
     */

    /** define shared pointer PoseConstraintsPlanner::Ptr*/
    using Ptr = std::shared_ptr<PoseConstraintsPlanner>;

    void setConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray &constraints);

    /* Set the start configuration and compute its pose
     * @param start_config The start joint configuration
     * @return true if the start configuration respects the pose constraints, false otherwise
     */
    bool setStartConfiguration(const Eigen::VectorXd& start_config);

    /* Set the goal configuration and compute its pose
     * @param goal_conf The goal joint configuration
     * @return true if the goal configuration respects the pose constraints, false otherwise
     */
    bool setGoalConfiguration(const Eigen::VectorXd& goal_conf);


    /* Set the goal pose and compute its IK solutions
     * @param goal_pose The goal end-effector pose
     * @return true if the goal pose respects the pose constraints, false otherwise
     */
    bool setGoalPose(const Eigen::Affine3d& goal_pose);

    /* Enable or disable logging
     * @param enable True to enable logging, false to disable
     */
    void setLogging(const bool& enable);

    /* Solve the planning problem within a given timeout
     * @param timeout The maximum time allowed for planning (in seconds)
     * @param solution The resulting path if a solution is found
     * @return true if a solution is found, false otherwise
     */
    bool solve(const double& timeout,
               graph::core::PathPtr& solution,
               info& planning_info);

  protected:
    Eigen::VectorXd start_config_;
    Eigen::Affine3d goal_pose_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ik_solver::IkSolver> ik_solver_;
    graph::core::CollisionCheckerPtr checker_;
    cnr_logger::TraceLoggerPtr logger_;
    graph::core::SamplerPtr sampler_;
    graph::core::MetricsPtr metrics_;
    unsigned int desired_solutions = 32;
    unsigned int min_stall_iterations = 100;
    unsigned int max_stall_iterations = 1000;
    ik_solver::Configurations goal_configurations_;
    std::string world_frame_ = "world";
    std::string tool_frame_ = "ur10_tool0";
    double max_distance = 0.1;
    bool use_kdtree = true;
    double goal_bias = 0.05;

    Eigen::Affine3d T_w_b; // world to base
    Eigen::Affine3d T_f_t; // flange to tool
    Eigen::Affine3d T_b_w; // base to world
    Eigen::Affine3d T_t_f; // tool to flange
    Eigen::Affine3d T_w_start; // world to start pose

    PoseConstraintsManager  pose_constraints_manager_;

    bool print_log_ = false;
  };

}  // namespace pose_constraints_planner
