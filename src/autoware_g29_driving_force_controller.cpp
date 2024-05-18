#include "autoware_g29_driving_force_controller.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

AutowareG29DrivingForceController::AutowareG29DrivingForceController(
    const std::string &name)
    : Node(name) {
  subscription_ = this->create_subscription<
      autoware_auto_vehicle_msgs::msg::SteeringReport>(
      "/vehicle/status/steering_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&AutowareG29DrivingForceController::targetCallback, this,
                std::placeholders::_1));
  readParameters();
  initDevice();
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / loop_rate_)),
      std::bind(&AutowareG29DrivingForceController::updateLoop, this));
}

AutowareG29DrivingForceController::~AutowareG29DrivingForceController() {
  if (device_handle_ >= 0) {
    effect_.type = FF_CONSTANT;
    effect_.id = -1;
    effect_.u.constant.level = 0;
    effect_.direction = 0;
    if (ioctl(device_handle_, EVIOCSFF, &effect_) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to upload effect_");
    }
    close(device_handle_);
  }
}

void AutowareG29DrivingForceController::readParameters() {
  device_name_ = declare_parameter<std::string>("device_name", "/dev/g29");
  loop_rate_ = declare_parameter<double>("loop_rate", 30.0);
  steering_handle_angle_ratio_ =
      declare_parameter<double>("steering_handle_angle_ratio", 16.0);
  max_torque_ = declare_parameter<double>("max_torque", 0.5);
  min_torque_ = declare_parameter<double>("min_torque", 0.16);
  kp_ = declare_parameter<double>("kp", 7.0);
  ki_ = declare_parameter<double>("ki", 100.0);
  kd_ = declare_parameter<double>("kd", 0.0);
}

void AutowareG29DrivingForceController::initDevice() {
  // setup device
  // unsigned char key_bits[1 + KEY_MAX / 8 / sizeof(unsigned char)];
  unsigned char abs_bits[1 + ABS_MAX / 8 / sizeof(unsigned char)];
  unsigned char ff_bits[1 + FF_MAX / 8 / sizeof(unsigned char)];
  struct input_event event;
  struct input_absinfo abs_info;

  device_handle_ = open(device_name_.c_str(), O_RDWR | O_NONBLOCK);
  if (device_handle_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "cannot open device : %s",
                 device_name_.c_str());
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(), "device opened : %s", device_name_.c_str());
  }

  // which axes has the device?
  memset(abs_bits, 0, sizeof(abs_bits));
  if (ioctl(device_handle_, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) <
      0) {
    RCLCPP_ERROR(this->get_logger(), "cannot get abs bits");
    rclcpp::shutdown();
  }

  // get some information about force feedback
  memset(ff_bits, 0, sizeof(ff_bits));
  if (ioctl(device_handle_, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
    RCLCPP_ERROR(this->get_logger(), "cannot get ff bits");
    rclcpp::shutdown();
  }

  // get axis value range
  if (ioctl(device_handle_, EVIOCGABS(axis_code_), &abs_info) < 0) {
    RCLCPP_ERROR(this->get_logger(), "cannot get axis range");
    rclcpp::shutdown();
  }

  axis_max_ = abs_info.maximum;
  axis_min_ = abs_info.minimum;
  if (axis_min_ >= axis_max_) {
    RCLCPP_ERROR(this->get_logger(), "axis range has bad value");
    rclcpp::shutdown();
  }

  // check force feedback is supported?
  if (!checkBit(FF_CONSTANT, ff_bits)) {
    RCLCPP_ERROR(this->get_logger(), "force feedback is not supported");
    rclcpp::shutdown();

  } else {
    RCLCPP_INFO(this->get_logger(), "force feedback supported");
  }

  // auto centering off
  memset(&event, 0, sizeof(event));
  event.type = EV_FF;
  event.code = FF_AUTOCENTER;
  event.value = 0;
  if (write(device_handle_, &event, sizeof(event)) != sizeof(event)) {
    RCLCPP_ERROR(this->get_logger(), "failed to disable auto centering");
    rclcpp::shutdown();
  }

  // init effect and get effect id
  memset(&effect_, 0, sizeof(effect_));
  effect_.type = FF_CONSTANT;
  effect_.id = -1; // initial value
  effect_.trigger.button = 0;
  effect_.trigger.interval = 0;
  effect_.replay.length = 0xffff; // longest value
  effect_.replay.delay = 0;       // delay from write(...)
  effect_.u.constant.level = 0;
  effect_.direction = 0xC000;
  effect_.u.constant.envelope.attack_length = 0;
  effect_.u.constant.envelope.attack_level = 0;
  effect_.u.constant.envelope.fade_length = 0;
  effect_.u.constant.envelope.fade_level = 0;

  if (ioctl(device_handle_, EVIOCSFF, &effect_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "failed to upload effect");
    rclcpp::shutdown();
  }

  // start effect_
  memset(&event, 0, sizeof(event));
  event.type = EV_FF;
  event.code = effect_.id;
  event.value = 1;
  if (write(device_handle_, &event, sizeof(event)) != sizeof(event)) {
    RCLCPP_ERROR(this->get_logger(), "failed to start effect");
    rclcpp::shutdown();
  }
}

void AutowareG29DrivingForceController::updateLoop() {
  struct input_event event;
  const double loop_period = 1.0 / loop_rate_;

  // get current state
  double current_position = 0.0;
  bool catch_event = false;
  while (read(device_handle_, &event, sizeof(event)) == sizeof(event)) {
    if (event.type == EV_ABS && event.code == axis_code_) {
      current_position = (event.value - (axis_max_ + axis_min_) * 0.5) * 2 /
                         (axis_max_ - axis_min_);
      catch_event = true;
    }
  }
  if (!catch_event) {
    current_position = previous_position_;
  }
  RCLCPP_INFO(this->get_logger(), "current position %f", current_position);
  RCLCPP_INFO(this->get_logger(), "target position %f", target_position_);

  double error = target_position_ - current_position;
  integral_error_ += error * loop_period;
  integral_error_ = std::clamp(integral_error_, -0.001, 0.001);

  double derivative_error = (error - previous_error_) * loop_period;
  derivative_error = std::clamp(derivative_error, -0.1, 0.1);

  double direction = error < 0.0 ? -1.0 : 1.0;
  RCLCPP_INFO(this->get_logger(), "kp torque_ %f", error * kp_);
  RCLCPP_INFO(this->get_logger(), "ki torque_ %f", integral_error_ * ki_);
  RCLCPP_INFO(this->get_logger(), "kd torque_ %f", derivative_error * kd_);

  double torque = error * kp_ + integral_error_ * ki_ + derivative_error * kd_;
  torque = std::clamp(std::fabs(torque), min_torque_, max_torque_) * direction;

  uploadEffect(torque, loop_period);

  previous_error_ = error;
  previous_position_ = current_position;
}

void AutowareG29DrivingForceController::uploadEffect(
    const double &torque, const double &attack_length) {
  effect_.u.constant.level = static_cast<__s16>(0x7fff * torque);
  RCLCPP_INFO(this->get_logger(), "torque %f", torque);

  effect_.direction = 0xC000;
  effect_.u.constant.envelope.attack_level = 0;
  effect_.u.constant.envelope.attack_length = static_cast<__u16>(attack_length);
  effect_.u.constant.envelope.fade_level = 0;
  effect_.u.constant.envelope.fade_length = static_cast<__u16>(attack_length);

  if (ioctl(device_handle_, EVIOCSFF, &effect_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to upload effect");
  }
}

void AutowareG29DrivingForceController::targetCallback(
    const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
  target_position_ = (msg->steering_tire_angle / (2.0 * M_PI)) *
                     steering_handle_angle_ratio_ * -1.0;
  target_position_ = std::clamp(target_position_, -1.0, 1.0);
}

int AutowareG29DrivingForceController::checkBit(int bit, unsigned char *array) {
  return (array[bit / (sizeof(unsigned char) * 8)] >>
          (bit % (sizeof(unsigned char) * 8))) &
         1;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutowareG29DrivingForceController>(
      "autoware_g29_driving_force_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
