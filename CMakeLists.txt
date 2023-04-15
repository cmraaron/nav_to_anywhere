cmake_minimum_required(VERSION 3.8)
project(nav_to_anywhere)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)

add_executable(nav_to_anywhere src/nav_to_anywhere.cpp)
ament_target_dependencies(nav_to_anywhere rclcpp rclcpp_action nav2_msgs nav_2d_utils)

target_include_directories(nav_to_anywhere PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav_to_anywhere PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS nav_to_anywhere
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(${PROJECT_NAME}-test test/test.cpp)
  ament_target_dependencies(${PROJECT_NAME}-test rclcpp)
endif()

ament_package()
