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
    duration_ = object.duration();
    double freq_step = (object.max_freq_pressing() - object.min_freq_pressing()) / 3.f;
    pressing_freq_.reserve(4);
    for (int i = 0; i < 4; i++) {
      double frequency = object.min_freq_pressing() + i * freq_step;
      pressing_freq_.emplace_back(frequency);
    }

    double depth_step = (object.max_depth() - object.min_depth()) / 3.f;
    for (int i = 0; i < 4; i++) {
      double depth = object.min_depth() + i * depth_step;
      depth_.emplace_back(depth);
    }

    freq_step = (object.max_freq_precision() - object.min_freq_precision()) / 3.f;
    precision_freq_.reserve(4);
    for (int i = 0; i < 4; i++) {
      double frequency = object.min_freq_precision() + i * freq_step;
      precision_freq_.emplace_back(frequency);
    }

    double angle_step = (object.max_angle() - object.min_angle()) / 3.f;
    angle_.reserve(4);
    for (int i = 0; i < 4; i++) {
      double angle = object.min_angle() + i * angle_step;
      angle_.emplace_back(angle);
    }

    freq_step = (object.max_freq_slipping() - object.min_freq_slipping()) / 3.f;
    for (int i = 0; i < 4; i++) {
      double frequency = object.min_freq_slipping() + i * freq_step;
      slip_freq_.emplace_back(frequency);
    }
    
    angle_step = (object.max_slip_angle() - object.min_slip_angle()) / 3.f;
    double amp_step = (object.max_slip_amp() - object.min_slip_amp()) / 3.f;
    slip_params_.reserve(4);
    for (int i = 0; i < 4; i++) {
      double angle = object.min_angle() + i * angle_step;
      double amp = object.min_slip_amp() + i * amp_step;
      slip_params_.emplace_back(std::make_pair(angle, amp));
    }

    // std::string file_name = path + object.object_name() + ".csv";
    // std::ofstream ofs(file_name);
    // if (!ofs.is_open()) {
    //   RCLCPP_ERROR(LOGGER, "Failed to open file: %s", file_name.c_str());
    // } else {
    //   ofs << "duration,depth,angle,slip_amp,slip_angle" << std::endl;
    //   for (int i = 0; i < object.repeat_num(); i++) {
    //     ofs << duration_ << "," << depth_[i] << "," << angle_[i] << "," << slip_amp_[i] << "," << slip_angle_ << std::endl;
    //   }
    //   ofs.close();
    // }
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
    int num_samples = duration_ * sample_rate_;
    double t = 0.0;
    t_.resize(num_samples);
    z_.resize(num_samples);
    dz_.resize(num_samples);
    twist.resize(num_samples);
    int idx_freq = idx % pressing_freq_.size();
    int idx_depth = idx / pressing_freq_.size();

    double frequency = pressing_freq_[idx_freq];
    double depth = depth_[idx_depth];
    RCLCPP_INFO(LOGGER, "freq: %f Pressing amp: %f", frequency, depth);
    for (int i = 0; i < num_samples; i++) {
      t_[i] = t;
      z_[i] = depth * 0.5 * std::cos(2.0 * M_PI * frequency * t);
      dz_[i] = -depth * M_PI * frequency * std::sin(2.0 * M_PI * frequency * t);
      twist[i] = std::make_unique<geometry_msgs::msg::Twist>();
      twist[i]->linear.z = dz_[i];
      t += dt_;
    }
    return true;
  }

  bool GeneratePrecisionTraj(std::vector<std::unique_ptr<geometry_msgs::msg::Twist>>& twist, int idx) {
    int idx_freq = idx % precision_freq_.size();
    int idx_angle = idx / precision_freq_.size();
    double frequency = precision_freq_[idx_freq];
    double angle = angle_[idx_angle];
    double period = 1.0 / frequency;
    double duration = duration_ + period * 0.5;
    int num_samples = duration * sample_rate_;
    double t = 0.0;
    t_.resize(num_samples);
    twist.resize(num_samples);
    RCLCPP_INFO(LOGGER, "freq: %f Rotation amp: %f", frequency, angle);
    for (int i = 0; i < num_samples; i++) {
      t_[i] = t;
      twist[i] = std::make_unique<geometry_msgs::msg::Twist>();
      // theta_x[i] = angle_[idx] * std::sin(2.0 * M_PI * t / period);
      twist[i]->angular.x = angle * 2.0 * M_PI * frequency * std::cos(2.0 * M_PI * frequency * t);
      if (i > sample_rate_ * period / 4 && i < (num_samples - sample_rate_ * period / 4)) {
        twist[i]->angular.y = angle * 2.0 * M_PI * frequency  * std::cos(2.0 * M_PI * frequency * t  - M_PI / 2);
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
    double idx_freq = idx % slip_freq_.size();
    double idx_params = idx / slip_freq_.size();
    double frequency = slip_freq_[idx_freq];
    double slip_angle = slip_params_[idx_params].first;
    double slip_amp = slip_params_[idx_params].second;
    
    double duration = duration_;
    int num_samples = duration * sample_rate_;
    double t = 0.0;
    t_.resize(num_samples * 3);
    twist.resize(num_samples * 3);

    RCLCPP_INFO(LOGGER, "freq: %f Slipppery amp: %f Slip angle %f", frequency, slip_amp, slip_angle);
    std::vector<double> slip_x(num_samples);
    std::vector<double> slip_y(num_samples);
    std::vector<double> slip_theta(num_samples);
    for (int i = 0; i < num_samples; i++) {
      t_[i] = t;
      twist[i] = std::make_unique<geometry_msgs::msg::Twist>();
      slip_x[i] = slip_amp * std::sin(2.0 * M_PI * t * frequency);
      twist[i]->linear.x = slip_amp * 2.0 * M_PI * frequency* std::cos(2.0 * M_PI * t * frequency);
      t += dt_;
    }
    for (int i = 0; i < num_samples; i++) {
      t_[i + num_samples] = t;
      twist[i + num_samples] = std::make_unique<geometry_msgs::msg::Twist>();
      slip_y[i] = slip_amp * std::sin(2.0 * M_PI * t * frequency);
      twist[i + num_samples]->linear.y = slip_amp * 2.0 * M_PI * frequency * std::cos(2.0 * M_PI * t * frequency);
      t += dt_;
    }
    for (int i = 0; i < num_samples; i++) {
      t_[i + num_samples * 2] = t;
      twist[i + num_samples * 2] = std::make_unique<geometry_msgs::msg::Twist>();
      slip_theta[i] = slip_angle * std::sin(2.0 * M_PI * t * frequency);
      twist[i + num_samples * 2]->angular.z = slip_angle * 2.0 * M_PI  * frequency * std::cos(2.0 * M_PI * t * frequency);
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
  std::vector<std::pair<double, double>> slip_params_; // slip angle and amplitude
  
  std::vector<double> depth_;
  std::vector<double> angle_;
  std::vector<double> pressing_freq_;
  std::vector<double> precision_freq_;
  std::vector<double> slip_freq_;


  double duration_ = 0.0;

};
    
}  // namespace fsr_experiments