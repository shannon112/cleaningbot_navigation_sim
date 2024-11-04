#include "cleaningbot_navigation_sim/robot_planner.h"

#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

float cross2d(const Eigen::Vector2f& vecA, const Eigen::Vector2f& vecB)
{
  return vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

RobotPlanner::RobotPlanner(std::shared_ptr<RobotVis> widget) : Node("robot_planner"), widget_(widget)
{
  timer_ = this->create_wall_timer(samplingTime_, std::bind(&RobotPlanner::timer_callback, this));
  service_ = this->create_service<cleaningbot_navigation_sim::srv::LoadPlanJson>(
      "load_plan_json", std::bind(&RobotPlanner::loadPlanJson, this, std::placeholders::_1, std::placeholders::_2));
}

void RobotPlanner::loadPlanJson(const std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Request> request,
                                std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Loading... %s", request->plan_json.c_str());
  const bool isSuccessfullyLoaded = parsePlanJson(request->plan_json);

  response->is_successfully_loaded = isSuccessfullyLoaded;
  response->num_path_samples = waypoints_.size();
  RCLCPP_INFO(this->get_logger(), "Loading is %s, there are %u waypoints",
              response->is_successfully_loaded ? "success" : "failed", response->num_path_samples);

  constructMap();
  widget_->setupVis(map_);
}

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
    if (waypoints_.size() < 2)
    {
      RCLCPP_ERROR(this->get_logger(), "Number of waypoints is lower than 2, not enough to compute.");
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

void RobotPlanner::constructMap()
{
  // origin is at top left
  map_.gridSize = mapGridSize_;

  float leftMost = std::numeric_limits<float>::max();
  float rightMost = std::numeric_limits<float>::min();
  float topMost = std::numeric_limits<float>::min();
  float bottomMost = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < waypoints_.size(); i++)
  {
    leftMost = std::min(leftMost, waypoints_[i][0]);
    rightMost = std::max(rightMost, waypoints_[i][0]);
    topMost = std::max(topMost, waypoints_[i][1]);
    bottomMost = std::min(bottomMost, waypoints_[i][1]);
  }

  std::vector<float> robotPointLens = { robotGadgetPoints_[0].norm(),  robotGadgetPoints_[1].norm(),
                                        robotContourPoints_[0].norm(), robotContourPoints_[1].norm(),
                                        robotContourPoints_[2].norm(), robotContourPoints_[3].norm() };
  float linkMaxLen = *std::max_element(robotPointLens.begin(), robotPointLens.end());
  const Eigen::Vector2f bottomleftMostPoint(leftMost - linkMaxLen, bottomMost - linkMaxLen);
  const Eigen::Vector2f topRightMostPoint(rightMost + linkMaxLen, topMost + linkMaxLen);

  const Eigen::Vector2i bottomleftMostGridIdx = (bottomleftMostPoint / map_.gridSize).cast<int>();  // floor
  const Eigen::Vector2i topRightGridIdx = (topRightMostPoint / map_.gridSize).cast<int>();          // floor

  map_.origin = bottomleftMostGridIdx.cast<float>() * map_.gridSize;
  const Eigen::Vector2i heightWidth = topRightGridIdx - bottomleftMostGridIdx + Eigen::Vector2i(1, 1);
  map_.grids = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(heightWidth[1], heightWidth[0]);
  RCLCPP_INFO(this->get_logger(), "Map origin (%f, %f), Dimension (%d, %d), Grid size %f", map_.origin[0],
              map_.origin[1], map_.grids.rows(), map_.grids.cols(), map_.gridSize);
}

float RobotPlanner::estimateVelocity()
{
  if (curIdx_ == 0)
  {
    return velocityMin_;  // assume that initial velocity is velocityMin_
  }
  else if (curIdx_ == waypoints_.size() - 1)
  {
    return 0.f;  // assume that robot stops at the end
  }
  else
  {
    const Eigen::Vector2f prevVec = waypoints_[curIdx_] - waypoints_[curIdx_ - 1];
    const Eigen::Vector2f nextVec = waypoints_[curIdx_ + 1] - waypoints_[curIdx_];
    const float theta = acos(prevVec.dot(nextVec) / (prevVec.norm() * nextVec.norm()));
    const float avgLen = (prevVec.norm() + nextVec.norm()) / 2.f;
    const float curvature = theta / avgLen;
    return std::isnan(curvature) ? prevStatus_.velocity : curvatureToVelocity(curvature);  // reuse prev vel if k is nan
                                                                                           // due to points too close
  }
}

float RobotPlanner::curvatureToVelocity(const float curvature)
{
  if (curvature < curvatureCritical_)
  {
    return velocityMax_;
  }
  else if (curvatureMax_ <= curvature)
  {
    return velocityMin_;
  }
  else
  {
    return velocityMax_ -
           (velocityMax_ - velocityMin_) * (curvature - curvatureCritical_) / (curvatureMax_ - curvatureCritical_);
  }
}

std::vector<Eigen::Vector2i> RobotPlanner::estimateNewCoveredGridIdsToNext()
{
  if (curIdx_ == waypoints_.size() - 1)
  {
    return {};
  }

  const Eigen::Vector2f translationVec = waypoints_[curIdx_ + 1] - waypoints_[curIdx_];
  const Eigen::Rotation2D<float> rotationMat(atan(translationVec[1] / translationVec[0]));

  // get four points of the covered rectangle area
  const Eigen::Vector2f bottomleft = rotationMat * robotGadgetPoints_[0] + waypoints_[curIdx_];
  const Eigen::Vector2f bottomright = rotationMat * robotGadgetPoints_[1] + waypoints_[curIdx_];
  const Eigen::Vector2f topleft = rotationMat * robotGadgetPoints_[0] + waypoints_[curIdx_ + 1];
  const Eigen::Vector2f topright = rotationMat * robotGadgetPoints_[1] + waypoints_[curIdx_ + 1];
  RCLCPP_INFO(this->get_logger(), "scanning area topleft (%f, %f)", topleft[0], topleft[1]);
  RCLCPP_INFO(this->get_logger(), "scanning area topright (%f, %f)", topright[0], topright[1]);
  RCLCPP_INFO(this->get_logger(), "scanning area bottomleft (%f, %f)", bottomleft[0], bottomleft[1]);
  RCLCPP_INFO(this->get_logger(), "scanning area bottomright (%f, %f)", bottomright[0], bottomright[1]);

  // get four lines (vectors)
  const Eigen::Vector2f rightVec = topright - bottomright;
  const Eigen::Vector2f topVec = topleft - topright;
  const Eigen::Vector2f leftVec = bottomleft - topleft;
  const Eigen::Vector2f bottomtVec = bottomright - bottomleft;

  // get bounding box for the rectangle
  const float maxX = std::max(std::max(topleft[0], topright[0]), std::max(bottomleft[0], bottomright[0]));
  const float minX = std::min(std::min(topleft[0], topright[0]), std::min(bottomleft[0], bottomright[0]));
  const float maxY = std::max(std::max(topleft[1], topright[1]), std::max(bottomleft[1], bottomright[1]));
  const float minY = std::min(std::min(topleft[1], topright[1]), std::min(bottomleft[1], bottomright[1]));

  const int maxXIdx = (maxX - map_.origin[0]) / map_.gridSize;
  const int minXIdx = (minX - map_.origin[0]) / map_.gridSize;
  const int maxYIdx = (maxY - map_.origin[1]) / map_.gridSize;
  const int minYIdx = (minY - map_.origin[1]) / map_.gridSize;

  // mark gridIds on map
  for (const auto gridId : prevStatus_.newCoveredGridIdsToNext)
  {
    map_.grids(gridId[1], gridId[0]) = true;  // rowId, colId
  }

  // get covered area for next iteration
  std::vector<Eigen::Vector2i> newCoveredGridIdsToNext;
  for (int x = minXIdx; x <= maxXIdx; x++)  // cols
  {
    for (int y = minYIdx; y <= maxYIdx; y++)  // rows
    {
      const Eigen::Vector2f gridCenter = map_.origin + Eigen::Vector2i(x, y).cast<float>() * map_.gridSize +
                                         Eigen::Vector2f(map_.gridSize * 0.5f, map_.gridSize * 0.5f);
      const bool isCovered =
          cross2d(rightVec, (gridCenter - bottomright)) >= 0.f && cross2d(topVec, (gridCenter - topright)) >= 0.f &&
          cross2d(leftVec, (gridCenter - topleft)) >= 0.f && cross2d(bottomtVec, (gridCenter - bottomleft)) >= 0.f;
      if (isCovered && !map_.grids(y, x))  // rowId, colId
      {
        newCoveredGridIdsToNext.push_back(Eigen::Vector2i(x, y));
      }
    }
  }

  return newCoveredGridIdsToNext;
}

std::array<Eigen::Vector2f, 4> RobotPlanner::estimateFootprint()
{
  std::array<Eigen::Vector2f, 4> footprint;
  const Eigen::Vector2f translationVec = waypoints_[curIdx_ + 1] - waypoints_[curIdx_];
  const Eigen::Rotation2D<float> rotationMat(std::atan2(translationVec[1], translationVec[0]));
  for (int i = 0; i < 4; i++)
  {
    footprint[i] = rotationMat * robotContourPoints_[i] + waypoints_[curIdx_];
  }
  return footprint;
}

std::array<Eigen::Vector2f, 2> RobotPlanner::estimateGadget()
{
  std::array<Eigen::Vector2f, 2> gadget;
  const Eigen::Vector2f translationVec = waypoints_[curIdx_ + 1] - waypoints_[curIdx_];
  const Eigen::Rotation2D<float> rotationMat(std::atan2(translationVec[1], translationVec[0]));
  gadget[0] = rotationMat * robotGadgetPoints_[0] + waypoints_[curIdx_];
  gadget[1] = rotationMat * robotGadgetPoints_[1] + waypoints_[curIdx_];
  return gadget;
}

void RobotPlanner::timer_callback()
{
  if (waypoints_.empty() || curIdx_ >= waypoints_.size())
  {
    return;  // waiting for data inputs
  }
  RCLCPP_INFO(this->get_logger(), "---reading dataframe %d---", curIdx_);

  Status curStatus;

  // position
  curStatus.position = waypoints_[curIdx_];
  RCLCPP_INFO(this->get_logger(), "position (%f, %f)", curStatus.position[0], curStatus.position[1]);

  // velocity
  curStatus.velocity = estimateVelocity();
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
  curStatus.newCoveredGridIdsToNext = estimateNewCoveredGridIdsToNext();
  curStatus.newCoveredAreaToNext = estimateNewCoveredGridIdsToNext().size() * std::pow(map_.gridSize, 2);
  curStatus.coveredAreaSoFar = prevStatus_.newCoveredAreaToNext + prevStatus_.coveredAreaSoFar;
  RCLCPP_INFO(this->get_logger(), "coveredAreaSoFar %f newCoveredAreaToNext %f", curStatus.coveredAreaSoFar,
              curStatus.newCoveredAreaToNext);

  // gadget
  curStatus.gadget = estimateGadget();
  RCLCPP_INFO(this->get_logger(), "gadget (%f, %f) (%f, %f)", curStatus.gadget[0][0], curStatus.gadget[0][1],
              curStatus.gadget[1][0], curStatus.gadget[1][1]);

  // footprint
  curStatus.footprint = estimateFootprint();
  RCLCPP_INFO(this->get_logger(), "footprint rear (%f, %f) (%f, %f)", curStatus.footprint[0][0],
              curStatus.footprint[0][1], curStatus.footprint[1][0], curStatus.footprint[1][1]);
  RCLCPP_INFO(this->get_logger(), "footprint front (%f, %f) (%f, %f)", curStatus.footprint[2][0],
              curStatus.footprint[2][1], curStatus.footprint[3][0], curStatus.footprint[3][1]);

  // vis
  widget_->updateVis(curStatus);

  // ending
  curIdx_++;
  prevStatus_ = curStatus;
}
