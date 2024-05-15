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
  loop_rate_ = declare_parameter<double>("loop_rate", 10.0);
  steering_handle_angle_ratio_ =
      declare_parameter<double>("steering_handle_angle_ratio", 17.0);
  max_torque_ = declare_parameter<double>("max_torque", 1.0);
  min_torque_ = declare_parameter<double>("min_torque", 0.2);
  brake_position_ = declare_parameter<double>("brake_position", 0.1);
  brake_torque_ = declare_parameter<double>("brake_torque", 0.2);
  auto_centering_max_torque_ =
      declare_parameter<double>("auto_centering_max_torque", 0.3);
  auto_centering_max_position_ =
      declare_parameter<double>("auto_centering_max_position", 0.2);
  epsilon_ = declare_parameter<double>("epsilon", 0.05);
  auto_centering_ = declare_parameter<bool>("auto_centering", false);
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
  is_target_updated_ = true;
  target_.position += 0.01;

  // get current state
  while (read(device_handle_, &event, sizeof(event)) == sizeof(event)) {
    if (event.type == EV_ABS && event.code == axis_code_) {
      position_ = (event.value - (axis_max_ + axis_min_) * 0.5) * 2 /
                  (axis_max_ - axis_min_);
    }
  }
  RCLCPP_INFO(this->get_logger(), "position %f", position_);
  RCLCPP_INFO(this->get_logger(), "target_.position %f", target_.position);

  if (is_brake_range_ || auto_centering_) {
    calculateCenteringForce(torque_, target_, position_);
    attack_length_ = 0.0;

  } else {
    calculateRotateForce(torque_, attack_length_, target_, position_);
    is_target_updated_ = false;
  }

  uploadEffect();
}

void AutowareG29DrivingForceController::calculateCenteringForce(
    double &torque, const Target &target, const double &current_position) {

  double diff = target.position - current_position;
  double direction = (diff > 0.0) ? 1.0 : -1.0;

  if (fabs(diff) < epsilon_) {
    torque = 0.0;

  } else {
    double torque_range = auto_centering_max_torque_ - min_torque_;
    double power =
        (fabs(diff) - epsilon_) / (auto_centering_max_position_ - epsilon_);
    double buf_torque = power * torque_range + min_torque_;
    torque = std::min(buf_torque, auto_centering_max_torque_) * direction;
  }
}

void AutowareG29DrivingForceController::calculateRotateForce(
    double &torque, double &attack_length, const Target &target,
    const double &current_position) {
  double diff = target.position - current_position;
  double direction = (diff > 0.0) ? 1.0 : -1.0;

  if (fabs(diff) < epsilon_) {
    torque = 0.0;
    attack_length = 0.0;

  } else if (fabs(diff) < brake_position_) {
    is_brake_range_ = true;
    torque = target.torque * brake_torque_ * -direction;
    attack_length = 1.0 / loop_rate_;

  } else {
    torque = target.torque * direction;
    attack_length = 1.0 / loop_rate_;
  }
}

void AutowareG29DrivingForceController::uploadEffect() {
  effect_.u.constant.level =
      static_cast<__s16>(0x7fff * std::min(torque_, max_torque_));
  effect_.direction = 0xC000;
  effect_.u.constant.envelope.attack_level = 0;
  effect_.u.constant.envelope.attack_length =
      static_cast<__u16>(attack_length_);
  effect_.u.constant.envelope.fade_level = 0;
  effect_.u.constant.envelope.fade_length = static_cast<__u16>(attack_length_);

  if (ioctl(device_handle_, EVIOCSFF, &effect_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to upload effect");
  }
}

void AutowareG29DrivingForceController::targetCallback(
    const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
  is_target_updated_ = true;
  target_.position =
      (msg->steering_tire_angle / (2.0 * M_PI)) * steering_handle_angle_ratio_;
  target_.position = std::clamp(target_.position, -1.0, 1.0);
  target_.torque = 0.3;
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
