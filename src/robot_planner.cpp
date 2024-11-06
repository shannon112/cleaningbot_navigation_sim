#include "cleaningbot_navigation_sim/robot_planner.h"
#include "cleaningbot_navigation_sim/robot_planner_utils.hpp"

#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

RobotPlanner::RobotPlanner(std::shared_ptr<RobotVis> widget) : Node("robot_planner"), widget_(widget)
{
  timer_ = this->create_wall_timer(samplingTime_, std::bind(&RobotPlanner::timer_callback, this));
  service_ = this->create_service<cleaningbot_navigation_sim::srv::LoadPlanJson>(
      "load_plan_json", std::bind(&RobotPlanner::loadPlanJson, this, std::placeholders::_1, std::placeholders::_2));
}

// service callback function, load and prepare all necessary info
void RobotPlanner::loadPlanJson(const std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Request> request,
                                std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Response> response)
{
  // load
  RCLCPP_INFO(this->get_logger(), "Loading... %s", request->plan_json.c_str());
  const bool isSuccessfullyLoaded = parsePlanJson(request->plan_json);

  response->is_successfully_loaded = isSuccessfullyLoaded;
  response->num_path_samples = waypoints_.size();
  RCLCPP_INFO(this->get_logger(), "Loading is %s, there are %u waypoints",
              response->is_successfully_loaded ? "success" : "failed", response->num_path_samples);
  if (!isSuccessfullyLoaded)
  {
    return;
  }

  // prepare
  waypoints_ = simplifyTrajectory(waypoints_, trajectoryDownSamplingDist_);
  RCLCPP_INFO(this->get_logger(), "After simplification, there are %zu waypoints", waypoints_.size());

  waypoints_ = resampleTrajectory(waypoints_, trajectoryUpSamplingNumIntervals_);
  RCLCPP_INFO(this->get_logger(), "After resampling, there are %zu waypoints", waypoints_.size());

  map_ = constructMap(robotContourPoints_, robotGadgetPoints_, waypoints_, mapGridSize_);
  RCLCPP_INFO(this->get_logger(), "Map origin (%f, %f), Dimension (%d, %d), Grid size %f", map_.origin[0],
              map_.origin[1], map_.grids.rows(), map_.grids.cols(), map_.gridSize);
  if (widget_)
  {
    widget_->setupVis(map_, waypoints_);
  }
}

// parse json file using nlohmann json
bool RobotPlanner::parsePlanJson(const std::string planJsonStr)
{
  std::ifstream file(planJsonStr);
  if (!file.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file");
    return false;
  }

  json jsonData;
  file >> jsonData;

  try
  {
    std::array<std::array<float, 2>, 4> robotContourPoints;
    robotContourPoints = jsonData["robot"];
    for (int i = 0; i < 4; i++)
    {
      robotContourPoints_[i] = Eigen::Vector2f(robotContourPoints[i][0], robotContourPoints[i][1]);
    }

    std::array<std::array<float, 2>, 2> robotGadgetPoints;
    robotGadgetPoints = jsonData["cleaning_gadget"];
    for (int i = 0; i < 2; i++)
    {
      robotGadgetPoints_[i] = Eigen::Vector2f(robotGadgetPoints[i][0], robotGadgetPoints[i][1]);
    }

    for (const std::array<float, 2>& point : jsonData["path"])
    {
      waypoints_.push_back(Eigen::Vector2f(point[0], point[1]));
    }
    if (waypoints_.size() < 2)  // assume at least two points
    {
      RCLCPP_ERROR(this->get_logger(), "Number of waypoints is lower than 2, not enough to compute.");
      return false;
    }
  }
  catch (json::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s", e.what());
    return false;
  }

  if (robotContourPoints_.size() > 0 && robotGadgetPoints_.size() > 0 && waypoints_.size() > 0)
  {
    RCLCPP_INFO(this->get_logger(), "read robot [[%f, %f], ...]", robotContourPoints_[0][0], robotContourPoints_[0][1]);
    RCLCPP_INFO(this->get_logger(), "read cleaning_gadget [[%f, %f], ...]", robotGadgetPoints_[0][0],
                robotGadgetPoints_[0][1]);
    RCLCPP_INFO(this->get_logger(), "read path [[%f, %f], ...]", waypoints_[0][0], waypoints_[0][1]);
  }
  return true;
}

// main loop for state estimation
void RobotPlanner::timer_callback()
{
  if (curIdx_ >= waypoints_.size())
  {
    clear();  // assume that we clean the data after finished
    return;
  }
  if (waypoints_.empty())
  {
    return;  // waiting for data inputs
  }
  RCLCPP_INFO(this->get_logger(), "---reading dataframe %d---", curIdx_);

  Status curStatus;

  // position
  curStatus.position = waypoints_[curIdx_];
  RCLCPP_INFO(this->get_logger(), "position (%f, %f)", curStatus.position[0], curStatus.position[1]);

  // velocity
  curStatus.velocity = estimateVelocity(waypoints_, curIdx_, curvatureApprxDist_, curvatureCritical_, curvatureMax_,
                                        velocityMin_, velocityMax_, prevStatus_.velocity);
  RCLCPP_INFO(this->get_logger(), "velocity %f", curStatus.velocity);

  // distnace
  curStatus.distanceToNext =
      (curIdx_ == waypoints_.size() - 1) ? 0.f : (waypoints_[curIdx_ + 1] - waypoints_[curIdx_]).norm();
  curStatus.distanceSoFar = prevStatus_.distanceToNext + prevStatus_.distanceSoFar;
  RCLCPP_INFO(this->get_logger(), "distanceToNext %f distanceSoFar %f", curStatus.distanceToNext,
              curStatus.distanceSoFar);

  // duration
  curStatus.durationToNext = (curIdx_ == waypoints_.size() - 1) ? 0.f : curStatus.distanceToNext / curStatus.velocity;
  curStatus.durationSoFar = prevStatus_.durationSoFar + prevStatus_.durationToNext;
  RCLCPP_INFO(this->get_logger(), "durationToNext %f durationSoFar %f", curStatus.durationToNext,
              curStatus.durationSoFar);

  // area
  curStatus.coveredGridIdsPrevToCur = prevStatus_.newCoveredGridIdsToNext;
  for (const auto gridId : prevStatus_.newCoveredGridIdsToNext)
  {
    map_.grids(gridId[1], gridId[0]) = true;  // rowId, colId
  }
  curStatus.coveredAreaSoFar = prevStatus_.newCoveredAreaToNext + prevStatus_.coveredAreaSoFar;
  curStatus.newCoveredGridIdsToNext = estimateNewCoveredGridIdsToNext(map_, waypoints_, curIdx_, robotGadgetPoints_);
  curStatus.newCoveredAreaToNext = curStatus.newCoveredGridIdsToNext.size() * std::pow(map_.gridSize, 2);
  RCLCPP_INFO(this->get_logger(), "coveredAreaSoFar %f newCoveredAreaToNext %f", curStatus.coveredAreaSoFar,
              curStatus.newCoveredAreaToNext);

  // gadget
  curStatus.gadget = estimateTransformedPoints(robotGadgetPoints_, waypoints_, curIdx_);
  RCLCPP_INFO(this->get_logger(), "gadget (%f, %f) (%f, %f)", curStatus.gadget[0][0], curStatus.gadget[0][1],
              curStatus.gadget[1][0], curStatus.gadget[1][1]);

  // footprint
  curStatus.footprint = estimateTransformedPoints(robotContourPoints_, waypoints_, curIdx_);
  RCLCPP_INFO(this->get_logger(), "footprint rear (%f, %f) (%f, %f)", curStatus.footprint[0][0],
              curStatus.footprint[0][1], curStatus.footprint[1][0], curStatus.footprint[1][1]);
  RCLCPP_INFO(this->get_logger(), "footprint front (%f, %f) (%f, %f)", curStatus.footprint[2][0],
              curStatus.footprint[2][1], curStatus.footprint[3][0], curStatus.footprint[3][1]);

  // vis
  if (widget_)
  {
    widget_->updateVis(curStatus);
  }

  // ending
  curIdx_++;
  prevStatus_ = curStatus;
}

void RobotPlanner::clear()
{
  robotContourPoints_ = {};
  robotGadgetPoints_ = {};
  waypoints_.clear();
  curIdx_ = 0;
  prevStatus_ = {};
  map_ = {};
}

const std::array<Eigen::Vector2f, 4>& RobotPlanner::getRobotContourPoints() const
{
  return robotContourPoints_;
}

const std::array<Eigen::Vector2f, 2>& RobotPlanner::getRobotGadgetPoints() const
{
  return robotGadgetPoints_;
}

const std::vector<Eigen::Vector2f>& RobotPlanner::getWaypoints() const
{
  return waypoints_;
}
