#pragma once

#include "ros_g29_force_feedback/msg/force_feedback.hpp"
#include <linux/input.h>
#include <rclcpp/rclcpp.hpp>

class AutowareG29DrivingForceController : public rclcpp::Node {
public:
  AutowareG29DrivingForceController(const std::string &name);
  virtual ~AutowareG29DrivingForceController();

private:
  void targetCallback(
      const ros_g29_force_feedback::msg::ForceFeedback::SharedPtr msg);
  void updateLoop();
  void initDevice();
  void calculateForce();
  void uploadEffect();
  int checkBit(int bit, unsigned char *array);
  void readParameters();

  rclcpp::Subscription<ros_g29_force_feedback::msg::ForceFeedback>::SharedPtr
      subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  int device_handle_;
  int axis_code_;
  int axis_min_, axis_max_;
  std::string device_name_;
  double loop_rate_;
  double max_torque_, min_torque_, brake_position_, brake_torque_;
  double auto_centering_max_torque_, auto_centering_max_position_, epsilon_;
  bool auto_centering_;
  ros_g29_force_feedback::msg::ForceFeedback current_target_;
  bool is_target_updated_;
  struct ff_effect effect_;
  double position_;
  double torque_;
  double attack_length_;
};
