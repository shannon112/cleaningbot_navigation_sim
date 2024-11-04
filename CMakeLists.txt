cmake_minimum_required(VERSION 3.5)
project(cleaningbot_navigation_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# customized msg/srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadPlanJson.srv"
)

# cleaningbot_sim node
set(cleaningbot_sim_HDRS
  include/cleaningbot_navigation_sim/robot_vis.h
)
qt5_wrap_cpp(cleaningbot_sim_MOCS ${cleaningbot_sim_HDRS})

add_executable(cleaningbot_sim 
  ${cleaningbot_sim_MOCS}
  src/robot_planner.cpp 
  src/robot_vis.cpp
  src/main.cpp
)

target_link_libraries(cleaningbot_sim 
  rclcpp::rclcpp
  ${EIGEN3_LIBRARIES}
  Qt5::Widgets                                   # Link Qt5 Widgets
  nlohmann_json::nlohmann_json                   # Link JSON library
)

target_include_directories(cleaningbot_sim PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
)

rosidl_target_interfaces(cleaningbot_sim
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS cleaningbot_sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

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
