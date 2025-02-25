cmake_minimum_required(VERSION 3.8)
project(autoware_nova_carter_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_nova_carter_interface SHARED
 src/nova_carter_interface.cpp
)

rclcpp_components_register_node(autoware_nova_carter_interface
 PLUGIN "autoware::nova_carter_interface::NovaCarterInterface"
 EXECUTABLE autoware_nova_carter_interface_node
)

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

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)