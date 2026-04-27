cmake_minimum_required(VERSION 3.20)
project(finger_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(modern_robotics REQUIRED)
find_package(fingerlib REQUIRED)
find_package(serial REQUIRED)

add_executable(sinusoid_demo src/sinusoid_demo.cpp)

target_include_directories(sinusoid_demo PUBLIC
  ${serial_INCLUDE_DIRS}
)

target_link_libraries(sinusoid_demo
  rclcpp::rclcpp
  fingerlib::fingerlib
  ${serial_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS sinusoid_demo
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()