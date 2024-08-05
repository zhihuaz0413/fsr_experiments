#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <numeric>
#include <filesystem>
#include <fstream>
#include <fcntl.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "geometry_msgs/msg/point_stamped.h"
#include "geometry_msgs/msg/twist.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_srvs/srv/empty.hpp"
#include "fsr_interfaces/msg/ni_force.hpp"
#include "fsr_interfaces/srv/trigger.hpp"
#include "fsr_interfaces/srv/stop_sig.hpp"
#include "fsr_interfaces/srv/record_force.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "exp_action.hpp"
#include "exp_conf.pb.h"

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
                                                    }
                                                    // If only solution a is successful, return a
                                                    else if (solution_a) {
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
    Initialize();
    moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75; // ??
    visual_tools_.publishText(text_pose, "Starting fsr experiments", rvt::WHITE, rvt::XLARGE);
    visual_tools_.trigger();
    package_share_directory_ = ament_index_cpp::get_package_share_directory("fsr_experiments") + "/config/";
  }
  virtual ~fsr_experiments() = default;

  void Execute(const std::string& conf) { // "/config/exp_conf.pb.txt"
    std::string conf_file = package_share_directory_ + conf;
    ExpConf exp_conf;
    int fd = open(conf_file.c_str(), O_RDONLY);
    if(fd >= 0) {
      google::protobuf::io::FileInputStream fileInput(fd);
      fileInput.SetCloseOnDelete(true);
      google::protobuf::TextFormat::Parse(&fileInput, &exp_conf);
      exp_conf.PrintDebugString();
    }
    else {
      RCLCPP_ERROR(LOGGER, "Failed to open file: %s", conf_file.c_str());
      return;
    }
    VisualizePrompt();

    RCLCPP_INFO(LOGGER, "Start FSR Experiment : %s", exp_conf.description().c_str());
    for (size_t i = 0; i < static_cast<size_t>(exp_conf.exp_group().size()); i++) {
      auto& exp_group = exp_conf.exp_group(i);
      output_dir_ = exp_conf.output_dir() + exp_group.group_name() + "/";
      if (!std::filesystem::exists(output_dir_)) {
        std::filesystem::create_directories(output_dir_);
      }
      for (size_t j = 0; j < static_cast<size_t>(exp_group.object_conf().size()); j++) {
        const auto& object_conf = exp_group.object_conf(j);
        SetupHome(exp_group.home_height());
        RCLCPP_INFO(LOGGER, "Start Experiment ****************** : %s", object_conf.object_name().c_str());
        VisualizePrompt();
        int repeat_num = object_conf.repeat_num();
        exp_action_ = std::make_unique<ExpAction>(object_conf, output_dir_);
        for (int k = 0; k < repeat_num; k++) {
          SetupHome(exp_group.start_height());
          SavePose(pose_dir_ + object_conf.object_name() + "_prepare_" + std::to_string(k) + ".csv");
          // VisualizePrompt();
          RCLCPP_INFO(LOGGER, "Start Experiment : %s, repeat: %d/%d ", object_conf.object_name().c_str(), k+1, repeat_num);
          std::string label = object_conf.object_name() + "_" + std::to_string(k);
          Calibrate(label);
          // VisualizePrompt();
          contaction_flag_ = false;
          SwitchController({"twist_controller"}, {"joint_trajectory_controller"});
          Move2Contact();
          SavePose(pose_dir_ + object_conf.object_name() + "_prepare_" + std::to_string(k) + ".csv");
          RCLCPP_INFO(LOGGER, "Connection established! ******** Ready for actions ********");
          ExecuteAction(object_conf, k);
          RCLCPP_INFO(LOGGER, "Experiment finished! ******** Ready for next ********");
          SwitchController({"joint_trajectory_controller"}, {"twist_controller"});
          StopRecording(label);
        }
      }
      SetupHome(exp_group.home_height());
    }
  }

 private:
   bool Initialize() {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    force_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    stop_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    fsr_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    controller_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    record_force_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options;
    options.callback_group = force_callback_group_;

    force_subscription_ = node_->create_subscription<fsr_interfaces::msg::NiForce>(
      "ni_force", rclcpp::SensorDataQoS(), std::bind(&fsr_experiments::NiForceCallBack, this, std::placeholders::_1), options);

    fsr_client_ptr_ = node_->create_client<fsr_interfaces::srv::Trigger>("trigger_recorder", rmw_qos_profile_services_default,
                                                                fsr_callback_group_);
    stop_sig_service_ = node_->create_service<fsr_interfaces::srv::StopSig>("stop_signal", 
                                                                std::bind(&fsr_experiments::StopMoving,
                                                                this, std::placeholders::_1, std::placeholders::_2), 
                                                                rmw_qos_profile_services_default, stop_callback_group_);

    record_force_client_ptr_ = node_->create_client<fsr_interfaces::srv::RecordForce>("record_force", rmw_qos_profile_services_default,
                                                                record_force_callback_group_);                                                        
    controller_manager_ptr_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller",
                                                                rmw_qos_profile_services_default, controller_callback_group_);

    twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("twist_controller/commands", 2);

    // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    return true;
  }

  void StopMoving(const std::shared_ptr<fsr_interfaces::srv::StopSig::Request> request, 
                  std::shared_ptr<fsr_interfaces::srv::StopSig::Response> response) {
    RCLCPP_INFO(LOGGER, "Stop moving! *****************************:%ld", request->mean_val);
    contaction_flag_ = true;
  }

  void ExecuteAction(const ObjectConf& conf, int idx) {
    if (exp_action_ == nullptr) {
      RCLCPP_ERROR(LOGGER, "Invalid ExpAction! *****************************");
      return;
    }
    for (size_t i = 0; i < static_cast<size_t>(conf.actions().size()); i++) {
      robot_state_ = moveit_cpp_ptr_->getCurrentState();
      RCLCPP_INFO(LOGGER, "Start Action : %s", conf.actions(i).c_str());
      double dt = 1.0 / conf.sample_rate();
      std::vector<std::unique_ptr<geometry_msgs::msg::Twist>> twist;
      if (!exp_action_->GetTrajectory(conf.actions(i), twist, idx)) {
        RCLCPP_ERROR(LOGGER, "Failed to generate trajectory for action: %s", conf.actions(i).c_str());
        return;
      }
      for (const auto& twist_msg : twist) {
        twist_publisher_->publish(*twist_msg);
        SaveCommand(pose_dir_ + conf.object_name() + "_cmd_" + conf.actions(i) + "_" + std::to_string(idx) + ".csv", twist_msg);
        int ms_dt = dt * 1000;
        rclcpp::sleep_for(std::chrono::milliseconds(ms_dt));
        SavePose(pose_dir_ + conf.object_name() + "_pose_" + conf.actions(i) + "_" + std::to_string(idx) + ".csv", true);
        // RCLCPP_INFO(LOGGER, "twist z: %f", twist_msg->linear.z);
      }
    }
  }

  void SetupHome(double height = 0.25) {
    //fsr_exp.Move2JointGoal(-3.0595408714496406, -0.527200532043311, 3.1168493338949776, 1.6578332344506774,
    //             0.01630129572138721, 0.9703866788252388, -1.4113049358717398);
    if (height < 0.05) {
      RCLCPP_ERROR(LOGGER, "Invalid height: %f", height);
      return;
    }
    std::unique_ptr<geometry_msgs::msg::PoseStamped> target_pose(std::make_unique<geometry_msgs::msg::PoseStamped>());
    target_pose->header.frame_id = "base_link";
    target_pose->pose.orientation.w = 0.003;
    target_pose->pose.orientation.x = 0.689;
    target_pose->pose.orientation.y = 0.725;
    target_pose->pose.orientation.z = -0.004;
    target_pose->pose.position.x = 0.48;
    target_pose->pose.position.y = 0.017;
    target_pose->pose.position.z = height;
    Move2Position(target_pose);
    pose_dir_ = output_dir_ + "kinova/";
    if (!std::filesystem::exists(pose_dir_)) {
      std::filesystem::create_directories(pose_dir_);
    }
  }

  void Move2Position(const std::unique_ptr<geometry_msgs::msg::PoseStamped>& target) {
    if (target == nullptr) {
      RCLCPP_ERROR(LOGGER, "Invalid pose: NULL!");
      return;
    }
    planning_components_ptr_->setGoal((*target), "end_effector_link");
    if (!Plan2Exe()) {
      RCLCPP_ERROR(LOGGER, "Failed to move to point, break!");
    }
  }

  bool Move2JointGoal(double const joint1, double const joint2, double const joint3,
                    double const joint4, double const joint5, double const joint6,
                    double const joint7) {
    VisualizePrompt();                        
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

  void Calibrate(const std::string& label) {
    RCLCPP_INFO(LOGGER, "Start Calibrating... ***************************** ");
    calibrate_flag_ = true;
    auto record_request = std::make_shared<fsr_interfaces::srv::RecordForce::Request>();
    record_request->record = true;
    record_request->labels = label;
    record_request->folder = output_dir_;
    auto record_result = record_force_client_ptr_->async_send_request(record_request);
    while (!record_force_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the record force service. Exiting.");
        break;
      }
      RCLCPP_INFO(LOGGER, "record force service not available, waiting again...");
    }
    if (record_result.get()->success) {
      RCLCPP_INFO(LOGGER, "Record force service called! --------------------------------- ");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to call service record_force !!! lease check the connection!");
    }

    auto request = std::make_shared<fsr_interfaces::srv::Trigger::Request>();
    request->record_flag = true;
    request->duration = 5;
    request->show_flag = true;
    request->folder = output_dir_;
    request->labels = label;
    auto fsr_result = fsr_client_ptr_->async_send_request(request);
    while (!fsr_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the fsr service. Exiting.");
        break;
      }
      RCLCPP_INFO(LOGGER, "fsr calibration service not available, waiting again...");
    }
    if (fsr_result.get()->success) {
      RCLCPP_INFO(LOGGER, "Calibration finished! ***************************** ");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to call service fsr_calibration: %s", fsr_result.get()->message.c_str());
    }
  }

  bool StopRecording(const std::string& label) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    auto record_request = std::make_shared<fsr_interfaces::srv::RecordForce::Request>();
    record_request->record = false;
    record_request->labels = label;
    record_request->folder = output_dir_;
    auto record_result = record_force_client_ptr_->async_send_request(record_request);
    while (!record_force_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the record force service. Exiting.");
        break;
      }
      RCLCPP_INFO(LOGGER, "record force service not available, waiting again...");
    }
    if (record_result.get()->success) {
      RCLCPP_INFO(LOGGER, "Record force service called! --------------------------------- ");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to call service record_force !!! lease check the connection!");
      return false;
    }

    auto request = std::make_shared<fsr_interfaces::srv::Trigger::Request>();
    request->record_flag = false;
    request->duration = 5;
    request->show_flag = false;
    request->folder = output_dir_;
    request->labels = label;
    auto fsr_result = fsr_client_ptr_->async_send_request(request);
    while (!fsr_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the fsr service. Exiting.");
        break;
      }
      RCLCPP_INFO(LOGGER, "fsr calibration service not available, waiting again...");
    }
    if (fsr_result.get()->success) {
      RCLCPP_INFO(LOGGER, "Stop fsr recording finished! ***************************** ");
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to call fsr stop recording service: %s", fsr_result.get()->message.c_str());
      return false;
    }
    return true;
  }

  void Move2Contact() {
    auto twist_msg = geometry_msgs::msg::Twist();
    while (contaction_flag_ == false) {
      // RCLCPP_INFO(LOGGER, "Waiting for connection...");
      twist_msg.linear.z = 0.002;
      twist_publisher_->publish(twist_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    if (contaction_flag_) {
      twist_msg.linear.z = 0.0;
      for (size_t i = 0; i < 100; i++) {
        twist_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
      }
      RCLCPP_INFO(LOGGER, "Connection established! ******** Ready for actions ********");
    }
  }

  bool SwitchController(const std::vector<std::string>& activate_controllers, const std::vector<std::string>& deactivate_controllers) {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = activate_controllers;
    request->deactivate_controllers = deactivate_controllers;
    request->strictness = 1;
    request->activate_asap = true;
    auto controller_result = controller_manager_ptr_->async_send_request(request);
    while (!controller_manager_ptr_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the controller service. Exiting.");
        break;
      }
      RCLCPP_INFO(LOGGER, "controller service not available, waiting again...");
    }
    if (controller_result.get()->ok) {
      RCLCPP_INFO(LOGGER, "Controller switched from '%s' to '%s'! ***************************** ",
                  deactivate_controllers[0].c_str(), activate_controllers[0].c_str());
      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to switch controller! :( ***************************** ");
      return false;
    }
  }

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

  void SavePose(const std::string& file_name, bool in_twist = false) {
    auto current_state = moveit_cpp_ptr_->getCurrentState();
    auto current_pose = current_state->getGlobalLinkTransform("tool_frame");
    if (in_twist) {
      current_pose = robot_state_->getGlobalLinkTransform("tool_frame").inverse() * current_pose;
    }
    auto euler = current_pose.rotation().eulerAngles(0, 1, 2);
    Eigen::Quaterniond quaternion(current_pose.rotation());

    std::ofstream file(file_name, std::ios::app);
    // time,pose x,y,z, orientation w,x,y,z, euler x,y,z
    if (file.is_open()) {
      file << node_->get_clock()->now().nanoseconds() << ", ";
      file << current_pose.translation().x() << ", " << current_pose.translation().y() << ", " << current_pose.translation().z() << ", ";
      file << (euler[0] * 180 / M_PI) << ", " << (euler[1] * 180 / M_PI) << ", " << (euler[2] * 180 / M_PI) << ", ";
      file << quaternion.w() << ", " << quaternion.x() << ", " << quaternion.y() << ", " << quaternion.z() << std::endl;
      file.close();
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to open file: %s", file_name.c_str());
    }
  }

  void SaveCommand(const std::string& file_name, const std::unique_ptr<geometry_msgs::msg::Twist>& command) {
    std::ofstream file(file_name, std::ios::app);
    if (file.is_open()) {
      file << node_->get_clock()->now().nanoseconds() << ", " << command->linear.x << ", " << command->linear.y << ", " << command->linear.z << ", ";
      file << command->angular.x << ", " << command->angular.y << ", " << command->angular.z << std::endl;
      file.close();
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to open file: %s", file_name.c_str());
    }
  }

  void NiForceCallBack(const fsr_interfaces::msg::NiForce & msg) {
    // RCLCPP_INFO_STREAM(LOGGER, "I heard: '" << msg.timestamp << "' : " << msg.force.x << ", " << msg.force.y << ", " << msg.force.z);
    // RCLCPP_INFO_STREAM(LOGGER, "I heard: torque : " << msg.torque.x << ", " << msg.torque.y << ", " << msg.torque.z);
    double force = sqrt(msg.force.z * msg.force.z + msg.force.y * msg.force.y + msg.force.x * msg.force.x);
    if (calibrate_flag_) {
      force_z_.emplace_back(force);
      if (force_z_.size() > 5000) {
        double sum = std::accumulate(force_z_.begin(), force_z_.end(), 0.0);
        mean_z_ = sum / force_z_.size();
        force_threshold_ = std::accumulate(force_z_.begin(), force_z_.end(), 0.0,
                                    [&](double accumulator, double value) {
                                      return accumulator + (value - mean_z_) * (value - mean_z_);
                                    });
        force_threshold_ = std::sqrt(force_threshold_ / force_z_.size());
        RCLCPP_INFO(LOGGER, "Calibration finished, mean force: %f, std: %f, force: %f", mean_z_, force_threshold_, msg.force.z);
        calibrate_flag_ = false;
        force_z_.clear();
        count_force_ = 0;
      }
    }
    if (!contaction_flag_) {
      //RCLCPP_INFO(LOGGER, "Ni force read: ----------------- : %f, %f", force, fabs(force - mean_z_));
      if (fabs(force - mean_z_)  > 17 * force_threshold_ + 2.5) {
        count_force_++;
        RCLCPP_INFO(LOGGER, "Extreme Contact detected! ************ : %f", force);
        if (count_force_ > 20) {
          contaction_flag_ = true;
        }
      }
      else {
        count_force_ = 0;
      }
    }
  }

  std::shared_ptr<rclcpp::Node> node_ = nullptr;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_ = nullptr;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_ptr_ = nullptr;
  moveit_visual_tools::MoveItVisualTools visual_tools_;
  moveit_msgs::msg::MotionPlanRequest planning_query_request_;
  std::vector<double> force_z_;
  double mean_z_ = 0.0;
  double force_threshold_ = 1.0;
  int count_force_ = 0;
  std::unique_ptr<ExpAction> exp_action_= nullptr;
  std::string output_dir_;
  std::string pose_dir_;

  moveit::core::RobotStatePtr robot_state_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr force_callback_group_ = nullptr;
  inline static bool contaction_flag_ = true;
  inline static bool calibrate_flag_ = false;
  rclcpp::CallbackGroup::SharedPtr fsr_callback_group_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr record_force_callback_group_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr controller_callback_group_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr stop_callback_group_ = nullptr;
  rclcpp::Client<fsr_interfaces::srv::Trigger>::SharedPtr fsr_client_ptr_;
  rclcpp::Service<fsr_interfaces::srv::StopSig>::SharedPtr stop_sig_service_;
  rclcpp::Client<fsr_interfaces::srv::RecordForce>::SharedPtr record_force_client_ptr_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_manager_ptr_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Subscription<fsr_interfaces::msg::NiForce>::SharedPtr force_subscription_;

  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string force_file_;

  std::string package_share_directory_;
};

} // namespace fsr_experiments