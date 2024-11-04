#ifndef ROBOT_PLANNER_H
#define ROBOT_PLANNER_H

#include "rclcpp/rclcpp.hpp"

#include "cleaningbot_navigation_sim/srv/load_plan_json.hpp"
#include "cleaningbot_navigation_sim/robot_vis.h"
#include "cleaningbot_navigation_sim/robot_status.h"
#include "cleaningbot_navigation_sim/occupancy_map.h"

#include <Eigen/Dense>
#include <chrono>

using namespace std::chrono_literals;

class RobotPlanner : public rclcpp::Node
{
public:
  RobotPlanner(std::shared_ptr<RobotVis> widget);
  void loadPlanJson(const std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Request> request,
                    std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Response> response);
  bool parsePlanJson(const std::string planJsonStr);
  void constructMap();
  float estimateVelocity();
  float curvatureToVelocity(const float curvature);
  std::vector<Eigen::Vector2i> estimateNewCoveredGridIdsToNext();
  std::array<Eigen::Vector2f, 2> estimateGadget();
  std::array<Eigen::Vector2f, 4> estimateFootprint();

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<cleaningbot_navigation_sim::srv::LoadPlanJson>::SharedPtr service_;
  std::shared_ptr<RobotVis> widget_;

  // inputs
  std::array<Eigen::Vector2f, 4> robotContourPoints_;
  std::array<Eigen::Vector2f, 2> robotGadgetPoints_;
  std::vector<Eigen::Vector2f> waypoints_;

  // data
  std::size_t curIdx_ = 0;
  Status prevStatus_;
  OccupancyMap map_;

  // configs
  const Eigen::Matrix3f initPose =
      Eigen::Matrix3f::Identity();  // assume that the given robot base_link is at origin with theta=0 pose
  const float curvatureCritical_ = 0.5f;
  const float curvatureMax_ = 10.f;
  const float velocityMin_ = 0.15f;
  const float velocityMax_ = 1.1f;
  const std::chrono::milliseconds samplingTime_ = 500ms;
  const float mapGridSize_ = 0.01f;
};

#endif  // ROBOT_PLANNER_H