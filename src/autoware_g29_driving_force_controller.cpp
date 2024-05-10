#include "autoware_g29_driving_force_controller.hpp"
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

AutowareG29DrivingForceController::AutowareG29DrivingForceController(
    const std::string &name)
    : Node(name), device_handle_(-1), position_(0.0), torque_(0.0),
      attack_length_(0.0), axis_code_(ABS_X), is_target_updated_(false) {
  subscription_ =
      this->create_subscription<ros_g29_force_feedback::msg::ForceFeedback>(
          "/ff_target", rclcpp::SystemDefaultsQoS(),
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
      RCLCPP_ERROR(this->get_logger(), "Failed to upload m_effect");
    }
    close(device_handle_);
  }
}

void AutowareG29DrivingForceController::readParameters() {
  this->declare_parameter<std::string>("device_name", "default_device");
  this->declare_parameter<double>("loop_rate", 100.0);
  this->declare_parameter<double>("max_torque", 1.0);
  this->declare_parameter<double>("min_torque", 0.1);
  this->declare_parameter<double>("brake_position", 0.2);
  this->declare_parameter<double>("brake_torque", 0.5);
  this->declare_parameter<double>("auto_centering_max_torque", 0.3);
  this->declare_parameter<double>("auto_centering_max_position", 0.4);
  this->declare_parameter<double>("epsilon", 0.05);
  this->declare_parameter<bool>("auto_centering", false);
}

void AutowareG29DrivingForceController::initDevice() {
  // setup device
  unsigned char key_bits[1 + KEY_MAX / 8 / sizeof(unsigned char)];
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
  if (ioctl(device_handle_, EVIOCGABS(m_axis_code), &abs_info) < 0) {
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
  while (read(device_handle_, &event, sizeof(event)) == sizeof(event)) {
    if (event.type == EV_ABS && event.code == axis_code_) {
      position_ = (event.value - (axis_max_ + axis_min_) * 0.5) * 2 /
                  (axis_max_ - axis_min_);
    }
  }

  if (auto_centering_ || is_target_updated_) {
    is_target_updated_ = false;
    calculateForce();
    uploadEffect();
  }
}

void AutowareG29DrivingForceController::calculateForce() {
  double diff = current_target_.position - position_;
  double direction = (diff > 0.0) ? 1.0 : -1.0;

  if (fabs(diff) < epsilon_) {
    torque_ = 0.0;
    attack_length_ = 0.0;
  } else {
    torque_ = current_target_.torque * direction;
    attack_length_ = loop_rate_;
  }
}

void AutowareG29DrivingForceController::uploadEffect() {
  effect_.u.constant.level = 0x7fff * std::min(torque_, max_torque_);
  effect_.direction = 0xC000;
  effect_.u.constant.envelope.attack_level = 0;
  effect_.u.constant.envelope.attack_length =
      static_cast<unsigned int>(attack_length_);
  effect_.u.constant.envelope.fade_level = 0;
  effect_.u.constant.envelope.fade_length =
      static_cast<unsigned int>(attack_length_);

  if (ioctl(device_handle_, EVIOCSFF, &effect_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to upload effect");
  }
}

void AutowareG29DrivingForceController::targetCallback(
    const ros_g29_force_feedback::msg::ForceFeedback::SharedPtr msg) {
  is_target_updated_ = true;
  current_target_ = *msg;
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
