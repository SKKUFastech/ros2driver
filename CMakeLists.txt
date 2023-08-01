cmake_minimum_required(VERSION 3.8)
project(ros2driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(ros2driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#######################this is added###################################

# Add the new node executable
add_executable(ros_to_driver src/ros_to_driver.cpp)
ament_target_dependencies(ros_to_driver rclcpp geometry_msgs)

# Add the ServoEnable node executable
add_executable(subscriber_set_node src/subscriber.cpp)
ament_target_dependencies(subscriber_set_node rclcpp)    # CHANGE

add_executable(subscriber_class_node src/subscriber_class.cpp)
ament_target_dependencies(subscriber_class_node rclcpp)  

# Install the executable to the "lib/<package_name>" directory
install(TARGETS
  ros_to_driver
  subscriber_set_node
  subscriber_class_node
  DESTINATION lib/${PROJECT_NAME})

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Servoenable42.msg"
  "msg/Movestop49.msg"
  "msg/Emergencystop50.msg"
  "msg/Movesingleaxisabspos52.msg"
  "msg/Movesingleaxisincpos53.msg"
  "msg/Movetolimit54.msg"
  "msg/Movevelocity55.msg"
  "msg/Positionabsoverride56.msg"
  "msg/Movepause88.msg"
  )

  rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

  target_link_libraries(subscriber_set_node "${cpp_typesupport_target}")
  target_link_libraries(subscriber_class_node "${cpp_typesupport_target}")
  ##########################################################################


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()