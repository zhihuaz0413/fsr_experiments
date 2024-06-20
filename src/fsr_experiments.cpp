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
  //fsr_exp.Execute("exp_conf.prototxt");
  fsr_exp.Execute("exp_conf_test.prototxt");
  
  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}