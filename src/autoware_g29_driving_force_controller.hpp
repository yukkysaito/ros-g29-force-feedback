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
  struct Target {
    double position;
    double torque;
  };
  void targetCallback(
      const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);

  void updateLoop();
  void initDevice();
  void calculateCenteringForce(double &torque, const Target &target,
                               const double &current_position);
  void calculateRotateForce(double &torque, double &attack_length,
                            const Target &target,
                            const double &current_position);
  void uploadEffect();
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
  double max_torque_, min_torque_, brake_position_, brake_torque_;
  double auto_centering_max_torque_, auto_centering_max_position_, epsilon_;
  double steering_handle_angle_ratio_;
  bool auto_centering_;
  bool is_target_updated_ = false;
  bool is_brake_range_ = false;

  struct ff_effect effect_;
  double position_ = 0.0;
  double torque_ = 0.0;
  double attack_length_ = 0.0;

  Target target_;
};
