#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point_stamped.h>

#include "fsr_experiments.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");

  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("fsr_experiments", "", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  fsr_experiments::fsr_experiments fsr_exp(node);
  // fsr_exp.Move2JointGoal(-3.0595408714496406, -0.527200532043311, 3.1168493338949776, 1.6578332344506774,
  //             0.01630129572138721, 0.9703866788252388, -1.4113049358717398);

  std::vector<std::unique_ptr<geometry_msgs::msg::PoseStamped>> targets;

  geometry_msgs::msg::PoseStamped target_pose0;
  target_pose0.header.frame_id = "base_link";
  target_pose0.pose.orientation.w = -0.0009229567446489309;
  target_pose0.pose.orientation.x = 0.6656445983517955;
  target_pose0.pose.orientation.y = 0.746190617631904;
  target_pose0.pose.orientation.z = -0.00523681132766913;
  target_pose0.pose.position.x = 0.4647454832180771;
  target_pose0.pose.position.y = -0.03269964959535717;
  target_pose0.pose.position.z = 0.3;
  targets.emplace_back(std::make_unique<geometry_msgs::msg::PoseStamped>(target_pose0));

  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.header.frame_id = "base_link";
  target_pose1.pose.orientation.w = -0.0009229567446489309;
  target_pose1.pose.orientation.x = 0.6656445983517955;
  target_pose1.pose.orientation.y = 0.746190617631904;
  target_pose1.pose.orientation.z = -0.00523681132766913;
  target_pose1.pose.position.x = 0.4647454832180771;
  target_pose1.pose.position.y = -0.03269964959535717;
  target_pose1.pose.position.z = 0.1;
  targets.emplace_back(std::make_unique<geometry_msgs::msg::PoseStamped>(target_pose1));

  geometry_msgs::msg::PoseStamped target_pose2;
  target_pose2.header.frame_id = "base_link";
  target_pose2.pose.orientation.w = -0.009455508992813264;
  target_pose2.pose.orientation.x = 0.6656445983517955;
  target_pose2.pose.orientation.y = 0.746190617631904;
  target_pose2.pose.orientation.z = -0.00523681132766913;
  target_pose2.pose.position.x = 0.4647454832180771;
  target_pose2.pose.position.y = -0.03269964959535717;
  target_pose2.pose.position.z = 0.3;
  targets.emplace_back(std::make_unique<geometry_msgs::msg::PoseStamped>(target_pose2));

  fsr_exp.StartExperiments(targets);

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}