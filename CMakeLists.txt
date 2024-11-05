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
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
)
qt5_wrap_cpp(cleaningbot_sim_MOCS include/cleaningbot_navigation_sim/robot_vis.h)
set(cleaningbot_sim_SRCS
  ${cleaningbot_sim_MOCS}
  src/robot_planner.cpp 
  src/robot_vis.cpp
)
set(cleaningbot_sim_LIBS
  rclcpp::rclcpp
  ${EIGEN3_LIBRARIES}
  Qt5::Widgets
  nlohmann_json::nlohmann_json
)

add_executable(cleaningbot_sim 
  src/main.cpp
  ${cleaningbot_sim_SRCS}
)

target_link_libraries(cleaningbot_sim 
  ${cleaningbot_sim_LIBS}
)

target_include_directories(cleaningbot_sim PUBLIC
  ${cleaningbot_sim_HDRS}
)

rosidl_target_interfaces(cleaningbot_sim
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS cleaningbot_sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

option(BUILD_TESTING "Enable testing" ON)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies() # disabled copyright, cppcheck, cpplint, lint_cmake, uncrustify, xmllint

  ament_add_gtest(cleaningbot_sim_test
    test/main.cpp
    test/robot_planner_test.cpp
    ${cleaningbot_sim_SRCS}
  )

  target_link_libraries(cleaningbot_sim_test 
    gtest 
    gtest_main
    ${cleaningbot_sim_LIBS}
  )

  target_include_directories(cleaningbot_sim_test PUBLIC
    ${cleaningbot_sim_HDRS}
  )

  rosidl_target_interfaces(cleaningbot_sim_test
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
endif()

ament_package()
