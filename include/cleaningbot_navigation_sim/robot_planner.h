#ifndef ROBOT_PLANNER_H
#define ROBOT_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "cleaningbot_navigation_sim/action/load_plan_json.hpp"
#include "cleaningbot_navigation_sim/robot_vis.h"
#include "cleaningbot_navigation_sim/robot_status.h"
#include "cleaningbot_navigation_sim/occupancy_map.h"

#include <Eigen/Dense>
#include <chrono>

using namespace std::chrono_literals;

class RobotPlanner : public rclcpp::Node
{
public:
  using LoadPlanJson = cleaningbot_navigation_sim::action::LoadPlanJson;
  using GoalHandleLoadPlanJson = rclcpp_action::ServerGoalHandle<LoadPlanJson>;

  RobotPlanner(std::shared_ptr<RobotVis> widget);
  bool parsePlanJson(const std::string planJsonStr);
  const std::array<Eigen::Vector2f, 4>& getRobotContourPoints() const;
  const std::array<Eigen::Vector2f, 2>& getRobotGadgetPoints() const;
  const std::vector<Eigen::Vector2f>& getWaypoints() const;

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const LoadPlanJson::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLoadPlanJson> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleLoadPlanJson> goal_handle);
  void execute(const std::shared_ptr<GoalHandleLoadPlanJson> goal_handle);
  void clear();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<cleaningbot_navigation_sim::action::LoadPlanJson>::SharedPtr action_server_;
  std::shared_ptr<RobotVis> widget_;

  // data
  std::array<Eigen::Vector2f, 4> robotContourPoints_;  // assume that robot's contour is always four points
  std::array<Eigen::Vector2f, 2> robotGadgetPoints_;   // assume that robot's gadget is always two points
  std::vector<Eigen::Vector2f> waypoints_;
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
  const float trajectoryDownSamplingDist_ = 2.f * mapGridSize_;
  const std::size_t trajectoryUpSamplingNumIntervals_ = 50;
};

#endif  // ROBOT_PLANNER_H