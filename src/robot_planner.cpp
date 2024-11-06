#include "cleaningbot_navigation_sim/robot_planner.h"
#include "cleaningbot_navigation_sim/robot_planner_utils.hpp"

#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

RobotPlanner::RobotPlanner(std::shared_ptr<RobotVis> widget) : Node("robot_planner"), widget_(widget)
{
  action_server_ = rclcpp_action::create_server<cleaningbot_navigation_sim::action::LoadPlanJson>(
      this, "load_plan_json", std::bind(&RobotPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotPlanner::handle_cancel, this, std::placeholders::_1),
      std::bind(&RobotPlanner::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse RobotPlanner::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                      std::shared_ptr<const LoadPlanJson::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with filename %s", goal->plan_json);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotPlanner::handle_cancel(const std::shared_ptr<GoalHandleLoadPlanJson> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotPlanner::handle_accepted(const std::shared_ptr<GoalHandleLoadPlanJson> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&RobotPlanner::execute, this, std::placeholders::_1), goal_handle }.detach();
}

void RobotPlanner::execute(const std::shared_ptr<GoalHandleLoadPlanJson> goal_handle)
{
  rclcpp::Rate loop_rate(samplingTime_);

  // action msg
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<LoadPlanJson::Feedback>();
  auto result = std::make_shared<LoadPlanJson::Result>();
  result->is_successfully_loaded = false;
  result->num_path_samples = 0;
  result->path_length = 0.f;
  result->path_time = 0.f;
  result->path_area = 0.f;

  // load
  RCLCPP_INFO(this->get_logger(), "Loading... %s", goal->plan_json.c_str());
  const bool isSuccessfullyLoaded = parsePlanJson(goal->plan_json);

  RCLCPP_INFO(this->get_logger(), "Loading is %s, there are %u waypoints", isSuccessfullyLoaded ? "success" : "failed",
              waypoints_.size());
  if (!isSuccessfullyLoaded)
  {
    goal_handle->succeed(result);
    clear();
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
    widget_->setupVis(map_, waypoints_);

  // main loop
  Status prevStatus;
  for (std::size_t curIdx = 0; curIdx < waypoints_.size() && rclcpp::ok(); ++curIdx)
  {
    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      clear();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "---reading dataframe %d---", curIdx);
    Status curStatus;
    feedback->iteration = curIdx;
    goal_handle->publish_feedback(feedback);

    // position
    curStatus.position = waypoints_[curIdx];
    RCLCPP_INFO(this->get_logger(), "position (%f, %f)", curStatus.position[0], curStatus.position[1]);

    // velocity
    curStatus.velocity = estimateVelocity(waypoints_, curIdx, curvatureApprxDist_, curvatureCritical_, curvatureMax_,
                                          velocityMin_, velocityMax_, prevStatus.velocity);
    RCLCPP_INFO(this->get_logger(), "velocity %f", curStatus.velocity);

    // distnace
    curStatus.distanceToNext =
        (curIdx == waypoints_.size() - 1) ? 0.f : (waypoints_[curIdx + 1] - waypoints_[curIdx]).norm();
    curStatus.distanceSoFar = prevStatus.distanceToNext + prevStatus.distanceSoFar;
    RCLCPP_INFO(this->get_logger(), "distanceToNext %f distanceSoFar %f", curStatus.distanceToNext,
                curStatus.distanceSoFar);

    // duration
    curStatus.durationToNext = (curIdx == waypoints_.size() - 1) ? 0.f : curStatus.distanceToNext / curStatus.velocity;
    curStatus.durationSoFar = prevStatus.durationSoFar + prevStatus.durationToNext;
    RCLCPP_INFO(this->get_logger(), "durationToNext %f durationSoFar %f", curStatus.durationToNext,
                curStatus.durationSoFar);

    // area
    curStatus.coveredGridIdsPrevToCur = prevStatus.newCoveredGridIdsToNext;
    for (const auto gridId : prevStatus.newCoveredGridIdsToNext)
      map_.grids(gridId[1], gridId[0]) = true;  // rowId, colId
    curStatus.coveredAreaSoFar = prevStatus.newCoveredAreaToNext + prevStatus.coveredAreaSoFar;
    curStatus.newCoveredGridIdsToNext = estimateNewCoveredGridIdsToNext(map_, waypoints_, curIdx, robotGadgetPoints_);
    curStatus.newCoveredAreaToNext = curStatus.newCoveredGridIdsToNext.size() * std::pow(map_.gridSize, 2);
    RCLCPP_INFO(this->get_logger(), "coveredAreaSoFar %f newCoveredAreaToNext %f", curStatus.coveredAreaSoFar,
                curStatus.newCoveredAreaToNext);

    // gadget
    curStatus.gadget = estimateTransformedPoints(robotGadgetPoints_, waypoints_, curIdx);
    RCLCPP_INFO(this->get_logger(), "gadget (%f, %f) (%f, %f)", curStatus.gadget[0][0], curStatus.gadget[0][1],
                curStatus.gadget[1][0], curStatus.gadget[1][1]);

    // footprint
    curStatus.footprint = estimateTransformedPoints(robotContourPoints_, waypoints_, curIdx);
    RCLCPP_INFO(this->get_logger(), "footprint rear (%f, %f) (%f, %f)", curStatus.footprint[0][0],
                curStatus.footprint[0][1], curStatus.footprint[1][0], curStatus.footprint[1][1]);
    RCLCPP_INFO(this->get_logger(), "footprint front (%f, %f) (%f, %f)", curStatus.footprint[2][0],
                curStatus.footprint[2][1], curStatus.footprint[3][0], curStatus.footprint[3][1]);

    if (widget_)
      widget_->updateVis(curStatus);

    // ending
    prevStatus = curStatus;
    loop_rate.sleep();
  }

  // result
  if (rclcpp::ok())
  {
    result->is_successfully_loaded = true;
    result->num_path_samples = waypoints_.size();
    result->path_length = prevStatus.distanceSoFar;
    result->path_time = prevStatus.durationSoFar;
    result->path_area = prevStatus.coveredAreaSoFar;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    clear();
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

void RobotPlanner::clear()
{
  robotContourPoints_ = {};
  robotGadgetPoints_ = {};
  waypoints_.clear();
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
