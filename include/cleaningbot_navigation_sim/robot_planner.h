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
  bool parsePlanJson(const std::string planJsonStr);
  const std::array<Eigen::Vector2f, 4>& getRobotContourPoints() const;
  const std::array<Eigen::Vector2f, 2>& getRobotGadgetPoints() const;
  const std::vector<Eigen::Vector2f>& getWaypoints() const;

private:
  void loadPlanJson(const std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Request> request,
                    std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Response> response);
  void timer_callback();
  void clear();

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
      Eigen::Matrix3f::Identity();  // assume that the given robot description is wrt (0,0) with theta=0
  const float curvatureCritical_ = 0.5f;
  const float curvatureMax_ = 10.f;
  const float curvatureApprxDist_ = 0.1f;
  const float velocityMin_ = 0.15f;
  const float velocityMax_ = 1.1f;
  const std::chrono::milliseconds samplingTime_ = 5ms;
  const float mapGridSize_ = 0.01f;
  const float trajectoryDownSamplingDist = 2.f * mapGridSize_;
  const float trajectoryUpSamplingDist = 0.5f * mapGridSize_;
};

#endif  // ROBOT_PLANNER_H