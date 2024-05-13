cmake_minimum_required(VERSION 3.14)
project(autoware_g29_driving_force_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wconversion)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()
set(dependencies
  rclcpp
  std_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ForceFeedback.msg"
  DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})

add_executable(autoware_g29_driving_force_controller_node src/autoware_g29_driving_force_controller.cpp)
ament_target_dependencies(autoware_g29_driving_force_controller_node ${dependencies})
rosidl_target_interfaces(autoware_g29_driving_force_controller_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_auto_package()
