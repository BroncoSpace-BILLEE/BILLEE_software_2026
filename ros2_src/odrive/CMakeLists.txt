cmake_minimum_required(VERSION 3.8)
project(odrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(odrive src/odrive.cpp)
ament_target_dependencies(odrive rclcpp std_msgs sensor_msgs)
target_include_directories(odrive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(odrive PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# --- ros2_control hardware interface plugin ---
add_library(odrive_hw_interface SHARED src/odrive_hw_interface.cpp)
target_include_directories(odrive_hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(odrive_hw_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface odrive_hw_plugin.xml)

install(TARGETS odrive
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS odrive_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_export_libraries(odrive_hw_interface)

ament_package()
