#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

namespace {
const rclcpp::Logger LOGGER = rclcpp::get_logger("fsr_experiments");
const std::string PLANNING_GROUP = "manipulator";
static const std::vector<std::string> CONTROLLERS = {1, "joint_trajectory_controller"};
}  // namespace

namespace fsr_experiments {

planning_interface::MotionPlanResponse getShortestSolution(
        const std::vector<planning_interface::MotionPlanResponse>& solutions) {
  RCLCPP_INFO(LOGGER, "#####################################################");
  RCLCPP_INFO(LOGGER, "###################### Results ######################");
  for (auto const& solution : solutions) {
    RCLCPP_INFO(LOGGER, "Planner '%s' returned '%s'", solution.planner_id.c_str(),
                moveit::core::errorCodeToString(solution.error_code).c_str());
    if (solution.trajectory) {
      RCLCPP_INFO(LOGGER, "Path length: '%f', Planning time: '%f'", robot_trajectory::pathLength(*solution.trajectory),
                  solution.planning_time);
    }
  }
  // Find trajectory with minimal path
  auto const shortest_solution = std::min_element(solutions.begin(), solutions.end(),
                                                  [](const planning_interface::MotionPlanResponse& solution_a,
                                                    const planning_interface::MotionPlanResponse& solution_b) {
                                                    // If both solutions were successful, check which path is shorter
                                                    if (solution_a && solution_b) {
                                                      return robot_trajectory::pathLength(*solution_a.trajectory) <
                                                            robot_trajectory::pathLength(*solution_b.trajectory);
                                                    } else if (solution_a) {
                                                      return true;
                                                    }
                                                    // Else return solution b, either because it is successful or not
                                                    return false;
                                                  });
  RCLCPP_INFO(LOGGER, "'%s' chosen as best solution (Shortest path)", shortest_solution->planner_id.c_str());
  RCLCPP_INFO(LOGGER, "#####################################################");
  return *shortest_solution;
}

class fsr_experiments {
 public:
  fsr_experiments() = delete;
  explicit fsr_experiments(const rclcpp::Node::SharedPtr node) 
    : node_{ node }
    , moveit_cpp_ptr_{ std::make_shared<moveit_cpp::MoveItCpp>(node) }
    , planning_components_ptr_{ std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr_) }
    , visual_tools_{ node, "base_link", "fsr_experiments", moveit_cpp_ptr_->getPlanningSceneMonitorNonConst() } {
    // rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(LOGGER, "Starting FSR experiments...");
    moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools_.publishText(text_pose, "Starting fsr experiments", rvt::WHITE, rvt::XLARGE);
    visual_tools_.trigger();
  }
  virtual ~fsr_experiments() = default;
  
  void StartExperiments(const std::vector<std::unique_ptr<geometry_msgs::msg::PoseStamped>>& targets) {
    VisualizePrompt();
    for (const auto& target_pose : targets) {
      if (target_pose == nullptr) {
        RCLCPP_ERROR(LOGGER, "Invalid pose: NULL!");
        continue;
      }
      planning_components_ptr_->setGoal((*target_pose), "end_effector_link");
      if (!Plan2Exe()) {
        RCLCPP_ERROR(LOGGER, "Failed to move to point, break!");
        break;
      }
      // Visualize the goal pose in Rviz
      visual_tools_.publishAxisLabeled(target_pose->pose, "target_pose");
      VisualizePrompt();
    }
  }

  bool Move2JointGoal(double const joint1, double const joint2, double const joint3,
                    double const joint4, double const joint5, double const joint6,
                    double const joint7) {
    auto robot_goal_state = planning_components_ptr_->getStartState();
    robot_goal_state->setJointPositions("joint_1", &joint1);
    robot_goal_state->setJointPositions("joint_2", &joint2);
    robot_goal_state->setJointPositions("joint_3", &joint3);
    robot_goal_state->setJointPositions("joint_4", &joint4);
    robot_goal_state->setJointPositions("joint_5", &joint5);
    robot_goal_state->setJointPositions("joint_6", &joint6);
    robot_goal_state->setJointPositions("joint_7", &joint7);

    // Set goal state
    planning_components_ptr_->setGoal(*robot_goal_state);
    planning_components_ptr_->setStartStateToCurrentState();
    return Plan2Exe();
  }

 private:
  bool VisualizePrompt () {
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools_.deleteAllMarkers();
    visual_tools_.trigger();
    return true;
  }

  bool Plan2Exe(){
    planning_components_ptr_->setStartStateToCurrentState();
    moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters multi_pipeline_plan_request{
      node_, { "ompl_rrtc", "pilz_lin", "chomp_planner", "ompl_rrt_star" }
    };
    auto plan_solution = planning_components_ptr_->plan(multi_pipeline_plan_request, &getShortestSolution);
    // Check if PlanningComponents succeeded in finding the plan
    if (plan_solution) {
      // Visualize the trajectory in Rviz
      auto robot_model_ptr = moveit_cpp_ptr_->getRobotModel();
      auto robot_start_state = planning_components_ptr_->getStartState();
      auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
      // Visualize the start pose in Rviz
      visual_tools_.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("end_effector_link"), "start_pose");
      visual_tools_.publishTrajectoryLine(plan_solution.trajectory, joint_model_group_ptr);
      visual_tools_.trigger();
          // Execute the trajectory and block until it's finished
      moveit_cpp_ptr_->execute(plan_solution.trajectory, CONTROLLERS);
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to find a plan for the given target pose");
      return false; 
    }
    return true;
  }

  std::shared_ptr<rclcpp::Node> node_ = nullptr;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_ = nullptr;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_ptr_ = nullptr;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  moveit_msgs::msg::MotionPlanRequest planning_query_request_;
};

} // namespace fsr_experiments
