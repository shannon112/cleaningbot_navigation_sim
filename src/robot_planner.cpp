#include "cleaningbot_navigation_sim/srv/load_plan_json.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using json = nlohmann::json;

float cross2d(const Eigen::Vector2f& vecA, const Eigen::Vector2f& vecB)
{
  return vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

struct OccupancyMap
{
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> grids;
  Eigen::Vector2f origin = { 0.f, 0.f };
  float gridSize = 0.01f;
  // the first grid including value at ([0, gridSize), [0, gridSize))
};

struct Status
{
  Eigen::Vector2f position = {};
  float velocity = 0.f;
  float distanceToNext = 0.f;
  float distanceSoFar = 0.f;
  float durationToNext = 0.f;
  float durationSoFar = 0.f;
  std::vector<Eigen::Vector2i> coveredGridIdsToNext;
  float newCoveredAreaToNext = 0.f;
  float coveredAreaSoFar = 0.f;
  std::array<Eigen::Vector2f, 4> footprints;
};

class RobotPlanner : public rclcpp::Node
{
public:
  RobotPlanner() : Node("robot_planner")
  {
    timer_ = this->create_wall_timer(samplingTime_, std::bind(&RobotPlanner::timer_callback, this));
    service_ = this->create_service<cleaningbot_navigation_sim::srv::LoadPlanJson>(
        "load_plan_json", std::bind(&RobotPlanner::loadPlanJson, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    // rclcpp::Time timestamp = this->get_clock()->now();
    // RCLCPP_INFO(this->get_logger(), "Message timestamp: %lf", timestamp.seconds());

    if (waypoints_.empty() || curIdx >= waypoints_.size())
    {
      return;  // waiting for data inputs
    }
    RCLCPP_INFO(this->get_logger(), "---reading dataframe %d---", curIdx);

    Status curStatus;

    // position
    curStatus.position = waypoints_[curIdx];
    RCLCPP_INFO(this->get_logger(), "position (%f, %f)", curStatus.position[0], curStatus.position[1]);

    // velocity
    if (curIdx == 0)
    {
      curStatus.velocity = velocityMin_;  // assume that initial velocity is velocityMin_
    }
    else if (curIdx == waypoints_.size() - 1)
    {
      curStatus.velocity = 0;  // assume that robot stops at the end
    }
    else
    {
      const Eigen::Vector2f prevVec = waypoints_[curIdx] - waypoints_[curIdx - 1];
      const Eigen::Vector2f nextVec = waypoints_[curIdx + 1] - waypoints_[curIdx];
      const float theta = acos(prevVec.dot(nextVec) / (prevVec.norm() * nextVec.norm()));
      const float avgLen = (prevVec.norm() + nextVec.norm()) / 2.f;
      const float curvature = theta / avgLen;
      curStatus.velocity = std::isnan(curvature) ?
                               prevStatus.velocity :
                               curvatureToVelocity(curvature);  // reuse prev vel if k is nan due to points too close
    }
    RCLCPP_INFO(this->get_logger(), "velocity %f", curStatus.velocity);

    // distnace
    const Eigen::Vector2f vecToNext =
        (curIdx == waypoints_.size() - 1) ? Eigen::Vector2f(0.f, 0.f) : waypoints_[curIdx + 1] - waypoints_[curIdx];
    curStatus.distanceToNext = vecToNext.norm();
    curStatus.distanceSoFar = prevStatus.distanceToNext + prevStatus.distanceSoFar;
    RCLCPP_INFO(this->get_logger(), "distanceToNext %f distanceSoFar %f", curStatus.distanceToNext,
                curStatus.distanceSoFar);

    // time
    curStatus.durationToNext = (curIdx == waypoints_.size() - 1) ? 0 : curStatus.distanceToNext / curStatus.velocity;
    curStatus.durationSoFar = prevStatus.durationSoFar + prevStatus.durationToNext;
    RCLCPP_INFO(this->get_logger(), "durationToNext %f durationSoFar %f", curStatus.durationToNext,
                curStatus.durationSoFar);

    // updatedGridIds
    if (curIdx != waypoints_.size() - 1)
    {
      Eigen::Vector2f translationVec = waypoints_[curIdx + 1] - waypoints_[curIdx];
      Eigen::Rotation2D<float> rotationMat(atan(translationVec[1] / translationVec[0]));

      // get four points of the covered rectangle area
      Eigen::Vector2f bottomleft = rotationMat * robotGadgetPoints_[0] + waypoints_[curIdx];
      Eigen::Vector2f bottomright = rotationMat * robotGadgetPoints_[1] + waypoints_[curIdx];
      Eigen::Vector2f topleft = rotationMat * robotGadgetPoints_[0] + waypoints_[curIdx + 1];
      Eigen::Vector2f topright = rotationMat * robotGadgetPoints_[1] + waypoints_[curIdx + 1];
      RCLCPP_INFO(this->get_logger(), "scanning area topleft (%f, %f)", topleft[0], topleft[1]);
      RCLCPP_INFO(this->get_logger(), "scanning area topright (%f, %f)", topright[0], topright[1]);
      RCLCPP_INFO(this->get_logger(), "scanning area bottomleft (%f, %f)", bottomleft[0], bottomleft[1]);
      RCLCPP_INFO(this->get_logger(), "scanning area bottomright (%f, %f)", bottomright[0], bottomright[1]);

      // get four lines (vectors)
      Eigen::Vector2f rightVec = topright - bottomright;
      Eigen::Vector2f topVec = topleft - topright;
      Eigen::Vector2f leftVec = bottomleft - topleft;
      Eigen::Vector2f bottomtVec = bottomright - bottomleft;

      // get bounding box for the rectangle
      float maxX = std::max(std::max(topleft[0], topright[0]), std::max(bottomleft[0], bottomright[0]));
      float minX = std::min(std::min(topleft[0], topright[0]), std::min(bottomleft[0], bottomright[0]));
      float maxY = std::max(std::max(topleft[1], topright[1]), std::max(bottomleft[1], bottomright[1]));
      float minY = std::min(std::min(topleft[1], topright[1]), std::min(bottomleft[1], bottomright[1]));

      int maxXIdx = (maxX - map.origin[0]) / map.gridSize;
      int minXIdx = (minX - map.origin[0]) / map.gridSize;
      int maxYIdx = (maxY - map.origin[1]) / map.gridSize;
      int minYIdx = (minY - map.origin[1]) / map.gridSize;
      RCLCPP_INFO(this->get_logger(), "bbx %d %d %d %d", maxXIdx, minXIdx, maxYIdx, minYIdx);

      // mark gridIds on map
      for (const auto gridId : prevStatus.coveredGridIdsToNext)
      {
        map.grids(gridId[0], gridId[1]) = true;
      }
      curStatus.coveredAreaSoFar = prevStatus.newCoveredAreaToNext + prevStatus.coveredAreaSoFar;

      // get covered area for next iteration
      for (int x = minXIdx; x <= maxXIdx; x++)
      {
        for (int y = minYIdx; y <= maxYIdx; y++)
        {
          Eigen::Vector2f gridCenter = map.origin + Eigen::Vector2i(x, y).cast<float>() * map.gridSize +
                                       Eigen::Vector2f(map.gridSize * 0.5f, map.gridSize * 0.5f);
          // RCLCPP_INFO(this->get_logger(), "gridCenter (%f, %f)", gridCenter[0], gridCenter[1]);
          bool isCovered =
              cross2d(rightVec, (gridCenter - bottomright)) >= 0.f && cross2d(topVec, (gridCenter - topright)) >= 0.f &&
              cross2d(leftVec, (gridCenter - topleft)) >= 0.f && cross2d(bottomtVec, (gridCenter - bottomleft)) >= 0.f;
          // RCLCPP_INFO(this->get_logger(), "isCovered %d", isCovered);
          if (isCovered && !map.grids(x, y))
          {
            // RCLCPP_INFO(this->get_logger(), "covered!");
            curStatus.coveredGridIdsToNext.push_back(Eigen::Vector2i(x, y));
            curStatus.newCoveredAreaToNext += std::pow(map.gridSize, 2);
          }
        }
      }
    }
    else
    {
      curStatus.coveredAreaSoFar = prevStatus.coveredAreaSoFar;
      curStatus.newCoveredAreaToNext = 0.f;
    }
    RCLCPP_INFO(this->get_logger(), "coveredAreaSoFar %f newCoveredAreaToNext %f", curStatus.coveredAreaSoFar,
                curStatus.newCoveredAreaToNext);

    // get four points for footprint
    Eigen::Vector2f translationVec = waypoints_[curIdx + 1] - waypoints_[curIdx];
    Eigen::Rotation2D<float> rotationMat(atan(translationVec[1] / translationVec[0]));
    for (int i = 0; i < 4; i++)
    {
      curStatus.footprints[i] = rotationMat * robotContourPoints_[i] + waypoints_[curIdx];
    }
    RCLCPP_INFO(this->get_logger(), "footprints (%f, %f)", curStatus.footprints[0][0], curStatus.footprints[0][1]);
    RCLCPP_INFO(this->get_logger(), "footprints (%f, %f)", curStatus.footprints[1][0], curStatus.footprints[1][1]);
    RCLCPP_INFO(this->get_logger(), "footprints (%f, %f)", curStatus.footprints[2][0], curStatus.footprints[2][1]);
    RCLCPP_INFO(this->get_logger(), "footprints (%f, %f)", curStatus.footprints[3][0], curStatus.footprints[3][1]);

    // ending
    curIdx++;
    prevStatus = curStatus;
  }

  bool parsePlanJson(const std::string planJsonStr)
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
      RCLCPP_INFO(this->get_logger(), "read robot [[%f, %f], ...]", robotContourPoints_[0][0],
                  robotContourPoints_[0][1]);
      RCLCPP_INFO(this->get_logger(), "read cleaning_gadget [[%f, %f], ...]", robotGadgetPoints_[0][0],
                  robotGadgetPoints_[0][1]);
      RCLCPP_INFO(this->get_logger(), "read path [[%f, %f], ...]", waypoints_[0][0], waypoints_[0][1]);
    }
    return true;
  }

  void loadPlanJson(const std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Request> request,
                    std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Loading... %s", request->plan_json.c_str());
    const bool isSuccessfullyLoaded = parsePlanJson(request->plan_json);

    response->is_successfully_loaded = isSuccessfullyLoaded;
    response->num_path_samples = waypoints_.size();
    RCLCPP_INFO(this->get_logger(), "Loading is %s, there are %u waypoints",
                response->is_successfully_loaded ? "success" : "failed", response->num_path_samples);

    constructMap();
  }

  void constructMap()
  {
    // origin is at top left
    map.gridSize = mapGridSize_;

    float leftMost = std::numeric_limits<float>::max();
    float rightMost = std::numeric_limits<float>::min();
    float topMost = std::numeric_limits<float>::min();
    float bottomMost = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < waypoints_.size(); i++)
    {
      leftMost = std::min(leftMost, waypoints_[i][0]);
      rightMost = std::max(rightMost, waypoints_[i][0]);
      topMost = std::min(bottomMost, waypoints_[i][1]);
      bottomMost = std::max(topMost, waypoints_[i][1]);
    }

    const float linkMaxLen = 10.f;  // std::max(robotGadgetPoints_[0].norm(), robotGadgetPoints_[1].norm());
    const Eigen::Vector2f bottomleftMostPoint(leftMost - linkMaxLen, bottomMost - linkMaxLen);
    const Eigen::Vector2f topRightMostPoint(rightMost + linkMaxLen, topMost + linkMaxLen);

    const Eigen::Vector2i bottomleftMostGridIdx = (bottomleftMostPoint / map.gridSize).cast<int>();  // floor
    const Eigen::Vector2i topRightGridIdx = (topRightMostPoint / map.gridSize).cast<int>();          // floor

    map.origin = bottomleftMostGridIdx.cast<float>() * map.gridSize;
    const Eigen::Vector2i heightWidth = topRightGridIdx - bottomleftMostGridIdx + Eigen::Vector2i(1, 1);
    map.grids = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(heightWidth[0], heightWidth[1]);
    RCLCPP_INFO(this->get_logger(), "Map origin (%f, %f), Dimension (%d, %d), Grid size %f", map.origin[0],
                map.origin[1], map.grids.rows(), map.grids.cols(), map.gridSize);
  }

  float curvatureToVelocity(const float curvature)
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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<cleaningbot_navigation_sim::srv::LoadPlanJson>::SharedPtr service_;

  // inputs
  std::array<Eigen::Vector2f, 4> robotContourPoints_;
  std::array<Eigen::Vector2f, 2> robotGadgetPoints_;
  std::vector<Eigen::Vector2f> waypoints_;

  // data
  std::size_t curIdx = 0;
  Status prevStatus;
  OccupancyMap map;

  // configs
  const Eigen::Matrix3f initPose =
      Eigen::Matrix3f::Identity();  // assume that the given robot base_link is at origin with theta=0 pose
  const float curvatureCritical_ = 0.5f;
  const float curvatureMax_ = 10.f;
  const float velocityMin_ = 0.15f;
  const float velocityMax_ = 1.1f;
  const std::chrono::milliseconds samplingTime_ = 10ms;
  const float mapGridSize_ = 0.01f;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPlanner>());
  rclcpp::shutdown();
  return 0;
}
