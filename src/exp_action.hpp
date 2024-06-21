#pragma once

#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include <random>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.h"
#include "rclcpp/rclcpp.hpp"
#include "exp_conf.pb.h"

namespace fsr_experiments {

const rclcpp::Logger LOGGER = rclcpp::get_logger("fsr_experiments");

class ExpAction {
 public:
  ExpAction() = delete;
  explicit ExpAction(const ObjectConf& object, const std::string& path) {
    actions_.assign(object.actions().begin(), object.actions().end());
    sample_rate_ = object.sample_rate();
    dt_ = 1.0 / sample_rate_;
    slip_angle_ = object.slip_angle();
    std::random_device rndsource;
    std::minstd_rand rndgen(rndsource());
    std::uniform_real_distribution<double> rand_depth(object.min_depth(), object.max_depth());
    std::uniform_real_distribution<double> rand_duration(object.min_duration(), object.max_duration());
    std::uniform_real_distribution<double> rand_angle(object.min_angle(), object.max_angle());
    std::uniform_real_distribution<double> rand_slip_amp(object.min_slip_amp(), object.max_slip_amp());
    depth_.resize(object.repeat_num());
    duration_.resize(object.repeat_num());
    angle_.resize(object.repeat_num());
    slip_amp_.resize(object.repeat_num());
    for (int i = 0; i < object.repeat_num(); i++) {
      duration_[i] = rand_duration(rndgen);
      depth_[i] = rand_depth(rndgen);
      angle_[i] = rand_angle(rndgen);
      slip_amp_[i] = rand_slip_amp(rndgen);
    }
    std::string file_name = path + object.object_name() + ".csv";
    std::ofstream ofs(file_name);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(LOGGER, "Failed to open file: %s", file_name.c_str());
    } else {
      ofs << "duration,depth,angle,slip_amp,slip_angle" << std::endl;
      for (int i = 0; i < object.repeat_num(); i++) {
        ofs << duration_[i] << "," << depth_[i] << "," << angle_[i] << "," << slip_amp_[i] << "," << slip_angle_ << std::endl;
      }
      ofs.close();
    }
  }

  virtual ~ExpAction() = default;

  bool GetTrajectory(const std::string& action, std::vector<std::unique_ptr<geometry_msgs::msg::Twist>>& twist, int idx) {
    if (action == "pressing") {
      return GeneratePressingTraj(twist, idx);
    } else if (action == "precision") {
      return GeneratePrecisionTraj(twist, idx);
    } else if (action == "slipping") {
      return GenerateSlipperyTraj(twist, idx);
    } else {
      RCLCPP_ERROR(LOGGER, "Invalid action: %s", action.c_str());
      return false;
    }
  }

 private:
  bool GeneratePressingTraj(std::vector<std::unique_ptr<geometry_msgs::msg::Twist>>& twist, int idx) {
    double duration = duration_[idx] * 1.2;
    int num_samples = duration * sample_rate_;
    double t = 0.0;
    t_.resize(num_samples);
    z_.resize(num_samples);
    dz_.resize(num_samples);
    twist.resize(num_samples);

    RCLCPP_INFO(LOGGER, "%d Pressing amp: %f", idx, depth_[idx]);
    for (int i = 0; i < num_samples; i++) {
      t_[i] = t;
      z_[i] = depth_[idx] * 0.5 * std::cos(2.0 * M_PI * t / duration_[idx]);
      dz_[i] = - M_PI * depth_[idx]  / duration_[idx] * std::sin(2.0 * M_PI * t / duration_[idx]);
      twist[i] = std::make_unique<geometry_msgs::msg::Twist>();
      twist[i]->linear.z = -dz_[i];
      t += dt_;
    }
    return true;
  }

  bool GeneratePrecisionTraj(std::vector<std::unique_ptr<geometry_msgs::msg::Twist>>& twist, int idx) {
    double duration = duration_[idx] * 1.5;
    int num_samples = duration * sample_rate_;
    double t = 0.0;
    t_.resize(num_samples);
    twist.resize(num_samples);
    RCLCPP_INFO(LOGGER, "%d Rotation amp: %f", idx, angle_[idx]);
    for (int i = 0; i < num_samples; i++) {
      t_[i] = t;
      twist[i] = std::make_unique<geometry_msgs::msg::Twist>();
      // theta_x[i] = angle_[idx] * std::sin(2.0 * M_PI * t / period);
      twist[i]->angular.x = angle_[idx] * 2.0 * M_PI / duration_[idx]  * std::cos(2.0 * M_PI * t / duration_[idx] );
      if (i > num_samples / 6 && i < num_samples / 6 * 5) {
        twist[i]->angular.y = angle_[idx] * 2.0 * M_PI / duration_[idx]  * std::cos(2.0 * M_PI * t / duration_[idx]  - M_PI / 2);
        // theta_y[i] = angle_[idx] * std::sin(2.0 * M_PI * t / period - M_PI / 2);
      } else {
        twist[i]->angular.y = 0.0;
        // theta_y[i] = 0.0;
      }
      t += dt_;
    }
    return true;
  }

  bool GenerateSlipperyTraj(std::vector<std::unique_ptr<geometry_msgs::msg::Twist>>& twist, int idx) {
    double duration = duration_[idx];
    int num_samples = duration * sample_rate_;
    double t = 0.0;
    t_.resize(num_samples * 3);
    twist.resize(num_samples * 3);

    RCLCPP_INFO(LOGGER, "%d Slipppery amp: %f", idx, slip_amp_[idx]);
    std::vector<double> slip_x(num_samples);
    std::vector<double> slip_y(num_samples);
    std::vector<double> slip_theta(num_samples);
    for (int i = 0; i < num_samples; i++) {
      t_[i] = t;
      twist[i] = std::make_unique<geometry_msgs::msg::Twist>();
      slip_x[i] = slip_amp_[idx] * std::sin(2.0 * M_PI * t / duration_[idx]);
      twist[i]->linear.x = slip_amp_[idx] * 2.0 * M_PI / duration_[idx] * std::cos(2.0 * M_PI * t / duration_[idx]);
      t += dt_;
    }
    for (int i = 0; i < num_samples; i++) {
      t_[i + num_samples] = t;
      twist[i + num_samples] = std::make_unique<geometry_msgs::msg::Twist>();
      slip_y[i] = slip_amp_[idx] * std::sin(2.0 * M_PI * t / duration_[idx]);
      twist[i + num_samples]->linear.y = slip_amp_[idx] * 2.0 * M_PI / duration_[idx] * std::cos(2.0 * M_PI * t / duration_[idx]);
      t += dt_;
    }
    for (int i = 0; i < num_samples; i++) {
      t_[i + num_samples * 2] = t;
      twist[i + num_samples * 2] = std::make_unique<geometry_msgs::msg::Twist>();
      slip_theta[i] = slip_angle_ * std::sin(2.0 * M_PI * t / duration_[idx]);
      twist[i + num_samples * 2]->angular.z = slip_angle_ * 2.0 * M_PI / duration_[idx] * std::cos(2.0 * M_PI * t / duration_[idx]);
      t += dt_;
    }
    return true;

  }

  int sample_rate_ = 100;
  double dt_ = 0.01;
  std::vector<double> t_;
  std::vector<double> z_;
  std::vector<double> dz_;

  std::vector<std::string> actions_;
  double slip_angle_ = 0.0;

  std::vector<double> duration_;
  std::vector<double> depth_;
  std::vector<double> angle_;
  std::vector<double> slip_amp_;

};
    
}  // namespace fsr_experiments