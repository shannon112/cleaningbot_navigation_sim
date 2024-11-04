cmake_minimum_required(VERSION 3.5)
project(cleaningbot_navigation_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

# customized msg/srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadPlanJson.srv"
)

# robot_planner node
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(robot_planner src/robot_planner.cpp)
target_include_directories(robot_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_planner rclcpp nlohmann_json Eigen3)
rosidl_target_interfaces(robot_planner
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

# robot_visualizer node

install(TARGETS robot_planner
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
