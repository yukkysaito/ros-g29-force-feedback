#pragma once

#include "autoware_g29_driving_force_controller/msg/force_feedback.hpp"
#include <linux/input.h>
#include <rclcpp/rclcpp.hpp>

class AutowareG29DrivingForceController : public rclcpp::Node {
public:
  AutowareG29DrivingForceController(const std::string &name);
  virtual ~AutowareG29DrivingForceController();

private:
  void targetCallback(
      const autoware_g29_driving_force_controller::msg::ForceFeedback::SharedPtr
          msg);
  void updateLoop();
  void initDevice();
  void calculateCenteringForce(
      double &torque,
      const autoware_g29_driving_force_controller::msg::ForceFeedback &target,
      const double &current_position);
  void calculateRotateForce(
      double &torque, double &attack_length,
      const autoware_g29_driving_force_controller::msg::ForceFeedback &target,
      const double &current_position);
  void uploadEffect();
  int checkBit(int bit, unsigned char *array);
  void readParameters();

  rclcpp::Subscription<
      autoware_g29_driving_force_controller::msg::ForceFeedback>::SharedPtr
      subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  int device_handle_ = -1;
  int axis_code_ = ABS_X;
  int axis_min_, axis_max_;
  std::string device_name_;
  double loop_rate_;
  double max_torque_, min_torque_, brake_position_, brake_torque_;
  double auto_centering_max_torque_, auto_centering_max_position_, epsilon_;
  bool auto_centering_;
  autoware_g29_driving_force_controller::msg::ForceFeedback current_target_;
  bool is_target_updated_ = false;
  bool is_brake_range_ = false;

  struct ff_effect effect_;
  double position_ = 0.0;
  double torque_ = 0.0;
  double attack_length_ = 0.0;
};
