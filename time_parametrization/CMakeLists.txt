cmake_minimum_required(VERSION 3.8)
project(time_parametrization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(time_parametrization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)  # for robot_model_loader
find_package(yaml-cpp REQUIRED)


add_executable(apply_time_parametrization_server src/time_parametrization_node.cpp)

ament_target_dependencies(apply_time_parametrization_server
        rclcpp
        rclcpp_action
        time_parametrization_msgs
        moveit_msgs
        moveit_core
        moveit_ros_planning
)

target_link_libraries(apply_time_parametrization_server yaml-cpp)

install(TARGETS apply_time_parametrization_server
        DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})


ament_package()
