#include <pose_constraints_planner/pose_constraints_planner.hpp>

#include <cmath>


namespace pose_constraints_planner
{

  PoseConstraintsPlanner::PoseConstraintsPlanner(rclcpp::Node::SharedPtr node, graph::core::CollisionCheckerPtr collision_checker, std::shared_ptr<ik_solver::IkSolver> ik_solver, graph::core::SamplerPtr sampler, graph::core::MetricsPtr metrics, cnr_logger::TraceLoggerPtr logger_ptr, std::string world_frame, std::string tool_frame)
    : node_(node),
      ik_solver_(ik_solver),
      checker_(collision_checker),
      logger_(logger_ptr),
      sampler_(sampler),
      metrics_(metrics)
  {
    world_frame_ = world_frame;
    tool_frame_ = tool_frame;
    std::string base_frame = ik_solver_->base_frame();
    std::string flange_frame = ik_solver_->flange_frame();

    RCLCPP_DEBUG_STREAM(node_->get_logger(),"PoseConstraintsPlanner Frames: world='"<<world_frame_<<"', base='"<<base_frame<<"', flange='"<<flange_frame<<"', tool='"<<tool_frame_<<"'");

    bool got_tf=ik_solver_->getTF(world_frame,base_frame,T_w_b);
    if (!got_tf)
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Failed to get TF from "<<world_frame<<" to "<<base_frame);

    got_tf=ik_solver_->getTF(flange_frame,tool_frame,T_f_t);
    if (!got_tf)
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Failed to get TF from "<<flange_frame<<" to "<<tool_frame);

    T_b_w = T_w_b.inverse();
    T_t_f = T_f_t.inverse();
  }

  void PoseConstraintsPlanner::setConstraints(const pose_constraints_msgs::msg::GeometricConstraintArray &constraints)
  {
    pose_constraints_manager_.setConstraints(constraints);
  }

  bool PoseConstraintsPlanner::setStartConfiguration(const Eigen::VectorXd &start_config)
  {
    T_w_start = ik_solver_->computeFk(start_config,
                                      T_w_b,
                                      T_f_t);
    if (!pose_constraints_manager_.checkConstraints(T_w_start,T_w_start))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Start configuration violates pose constraints.");
      return false;
    }
    start_config_ = start_config;
    return true;
  }

  bool PoseConstraintsPlanner::setGoalConfiguration(const Eigen::VectorXd &goal_conf)
  {
    Eigen::Affine3d T_w_goal = ik_solver_->computeFk(goal_conf,
                                                     T_w_b,
                                                     T_f_t);
    if (!pose_constraints_manager_.checkConstraints(T_w_goal,T_w_start))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Goal configuration violates pose constraints.");
      return false;
    }
    goal_pose_ = T_w_goal;
    goal_configurations_.clear();
    goal_configurations_.push_back(goal_conf);
    return true;
  }

  bool PoseConstraintsPlanner::setGoalPose(const Eigen::Affine3d &goal_pose)
  {
    if (!pose_constraints_manager_.checkConstraints(goal_pose,T_w_start))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Goal configuration violates pose constraints.");
      return false;
    }
    goal_pose_ = goal_pose;

    // EXAMPLE: compute inverse kinematics
    ik_solver::Configurations seeds;
    goal_configurations_ = ik_solver_->computeIk(goal_pose_,
                                                 T_w_b,
                                                 T_f_t,
                                                 seeds,
                                                 desired_solutions,
                                                 min_stall_iterations,
                                                 max_stall_iterations).configurations();

    return true;
  }

  void PoseConstraintsPlanner::setLogging(const bool &enable)
  {
    print_log_ = enable;
  }

  bool PoseConstraintsPlanner::solve(const double &timeout, graph::core::PathPtr &solution, info &planning_info)
  {


    std::random_device rd;  // Seed source
    std::mt19937 gen(rd()); // Mersenne Twister engine

    // Distributions for random sampling the goal bias and goal configurations
    std::uniform_real_distribution<> dis(0.0, 1.0); // for goal bias
    std::uniform_int_distribution<> dis_int(0,goal_configurations_.size() - 1); // for goal configuration index

    graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_config_,logger_);

    graph::core::NodePtr new_node;
    graph::core::TreePtr tree = std::make_shared<graph::core::Tree>(start_node,
                                                                    max_distance,
                                                                    checker_,
                                                                    metrics_,
                                                                    logger_,
                                                                    use_kdtree);
    auto start_time_rrt = std::chrono::steady_clock::now();
    auto start_time_rrt_i = std::chrono::steady_clock::now();
    auto end_time_rrt_i = std::chrono::steady_clock::now();
    double max_time_rrt = 0.0;
    double min_time_rrt = 0.0;
    double elapsed_rrt_i = 0.0;
    double reject_time = 0.0; // time spent to reject samples
    std::vector<double> rrt_iteration_times;

    int total_iterations = 0;
    int rrt_rejections = 0;
    int extension_rejections = 0;
    int total_rejections = 0;
    Eigen::VectorXd qrand;

    int cycles=0;
    int nodes=0;

    bool found_solution = false;

    while (rclcpp::ok())
    {

      std::stringstream report;
      if ((std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_rrt).count()) > timeout)
      {
        RCLCPP_WARN(node_->get_logger(),"Timeout reached, no solution found");
        break;
      }


      // RRT algorithm implementation

      double random_value = dis(gen);
      if (random_value<goal_bias)
      {
        // pick a random goal configuration
        int goal_index = dis_int(gen);
        qrand=goal_configurations_[goal_index];
      }
      else
      {
        total_iterations++;

        qrand=sampler_->sample();

        Eigen::Affine3d T_w_rand=ik_solver_->computeFk(qrand,
                                                       T_w_b,
                                                       T_f_t); // transformation from base to tool in qrand;

        if (!pose_constraints_manager_.checkConstraints(T_w_rand,T_w_start,&report))
        {
          rrt_rejections++;
          total_rejections++;
          // measure end time of each RRT iteration
          end_time_rrt_i = std::chrono::steady_clock::now();
          elapsed_rrt_i = std::chrono::duration<double, std::milli>(end_time_rrt_i - start_time_rrt_i).count();
          reject_time += elapsed_rrt_i;
          rrt_iteration_times.push_back(elapsed_rrt_i);
          max_time_rrt = std::max(max_time_rrt, elapsed_rrt_i);
          min_time_rrt = (min_time_rrt == 0.0) ? elapsed_rrt_i : std::min(min_time_rrt, elapsed_rrt_i);
          continue;
        }
        // LOG every 1000 iterations of feasible points
        if (cycles++>1000 && print_log_)
        {
          RCLCPP_INFO_STREAM(node_->get_logger(),"Start matrix:\n"<<T_w_start.matrix());
          RCLCPP_INFO_STREAM(node_->get_logger(),"Current matrix:\n"<<T_w_rand.matrix());
          RCLCPP_INFO_STREAM(node_->get_logger(),"Current point: "<<T_w_rand.translation().transpose());
          RCLCPP_INFO(node_->get_logger(),"Added %d nodes",nodes);
          RCLCPP_INFO_STREAM(node_->get_logger(),"Tree extended, "<<nodes<<" nodes in the tree");
          cycles=0;
        }
      }


      if (tree->extend(qrand, new_node))
      {
        auto new_conf = new_node->getConfiguration();
        Eigen::Affine3d T_w_new = ik_solver_->computeFk(new_conf,
                                                        T_w_b,
                                                        T_f_t);
        RCLCPP_DEBUG_STREAM(node_->get_logger(),"New node added: "<<new_conf.transpose());
        if(!pose_constraints_manager_.checkConstraints(T_w_new,
                                                       T_w_start,
                                                       &report))
        {
          total_rejections++;
          extension_rejections++;

          // Log every 200 rejections
          if(extension_rejections%200==0  && print_log_)
          {
            RCLCPP_ERROR(node_->get_logger(),"New configuration violates geometric constraints.");
            RCLCPP_ERROR_STREAM(node_->get_logger(),"Report:\n"<<report.str());
            RCLCPP_INFO(node_->get_logger(),"Added %d nodes",nodes);
            RCLCPP_INFO_STREAM(node_->get_logger(),"Tree extended, "<<nodes<<" nodes in the tree");
          }

          tree->removeNode(new_node);
          // measure end time of each RRT iteration
          end_time_rrt_i = std::chrono::steady_clock::now();
          elapsed_rrt_i = std::chrono::duration<double, std::milli>(end_time_rrt_i - start_time_rrt_i).count();
          reject_time += elapsed_rrt_i;
          rrt_iteration_times.push_back(elapsed_rrt_i);
          max_time_rrt = std::max(max_time_rrt, elapsed_rrt_i);
          min_time_rrt = (min_time_rrt == 0.0) ? elapsed_rrt_i : std::min(min_time_rrt, elapsed_rrt_i);
          continue;
        }

        nodes++;

        for (auto goal_conf : goal_configurations_)
        {
          if ((new_node->getConfiguration()-goal_conf).norm()<max_distance)
          {
            RCLCPP_INFO(node_->get_logger(),"Checking if tree can reach goal");
            if (checker_->checkConnection(new_node->getConfiguration(),goal_conf))
            {
              graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal_conf,logger_);
              tree->extend(qrand, goal_node);
              RCLCPP_INFO(node_->get_logger(),"Goal reached");

              solution = std::make_shared<graph::core::Path>(tree->getConnectionToNode(goal_node), metrics_, checker_, logger_);
              solution->setTree(tree);
              found_solution = true;
              break;
            }
          }
        }
      }
      // measure end time of each RRT iteration
      end_time_rrt_i = std::chrono::steady_clock::now();
      elapsed_rrt_i = std::chrono::duration<double, std::milli>(end_time_rrt_i - start_time_rrt_i).count();
      rrt_iteration_times.push_back(elapsed_rrt_i);
      max_time_rrt = std::max(max_time_rrt, elapsed_rrt_i);
      min_time_rrt = (min_time_rrt == 0.0) ? elapsed_rrt_i : std::min(min_time_rrt, elapsed_rrt_i);

    }

    planning_info.total_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_rrt).count();
    planning_info.total_iterations = total_iterations;
    planning_info.total_rejections = total_rejections;
    planning_info.max_iteration_time = max_time_rrt;
    planning_info.min_iteration_time = min_time_rrt;
    planning_info.average_iteration_time = (std::accumulate(rrt_iteration_times.begin(), rrt_iteration_times.end(), 0.0) / rrt_iteration_times.size());
    planning_info.total_reject_time = reject_time;

    return found_solution;
  }


}  // namespace pose_constraints_planner
