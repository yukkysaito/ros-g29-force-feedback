#pragma once

#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
// #include "autoware_g29_driving_force_controller/msg/force_feedback.hpp"
#include <linux/input.h>
#include <rclcpp/rclcpp.hpp>

class AutowareG29DrivingForceController : public rclcpp::Node {
public:
  AutowareG29DrivingForceController(const std::string &name);
  virtual ~AutowareG29DrivingForceController();

private:
  void targetCallback(
      const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);

  void updateLoop();
  void initDevice();
  void uploadEffect(const double &torque, const double &attack_length);
  int checkBit(int bit, unsigned char *array);
  void readParameters();

  rclcpp::Subscription<
      autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  int device_handle_ = -1;
  int axis_code_ = ABS_X;
  int axis_min_, axis_max_;
  std::string device_name_;
  double loop_rate_;
  double min_torque_, max_torque_;
  double kp_, ki_, kd_;
  double steering_handle_angle_ratio_;

  struct ff_effect effect_;
  double previous_position_ = 0.0;
  double target_position_ = 0.0;
  double torque_ = 0.0;
  double integral_error_ = 0.0;
  double previous_error_ = 0.0;
};
