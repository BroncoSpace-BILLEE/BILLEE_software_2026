cmake_minimum_required(VERSION 3.8)
project(roboclaw_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Add libroboclaw as subdirectory
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(../../roboclaw/libroboclaw libroboclaw)

add_library(roboclaw_driver SHARED src/roboclaw_driver.cpp)
target_include_directories(roboclaw_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../roboclaw/libroboclaw/include>
)
target_compile_features(roboclaw_driver PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(roboclaw_driver hardware_interface pluginlib rclcpp)
# Link to libroboclaw
target_link_libraries(roboclaw_driver libroboclaw ${Boost_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

install(TARGETS roboclaw_driver
  DESTINATION lib/${PROJECT_NAME})

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME})

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
