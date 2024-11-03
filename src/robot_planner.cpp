#include "cleaningbot_navigation_sim/srv/load_plan_json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;
using Point2F = std::array<float, 2>;
using Vec2F = std::array<float, 2>;
using Point2I = std::array<int, 2>;

Vec2F getVec2F(const Point2F& a, const Point2F& b)
{
  return { b[0] - a[0], b[1] - a[1] };
}

float dot(const Vec2F& vecA, const Vec2F& vecB)
{
  return vecA[0] * vecB[0] + vecA[1] * vecB[1];
}

float cross(const Vec2F& vecA, const Vec2F& vecB)
{
  return vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

float len(const Vec2F& vec)
{
  return sqrt(pow(vec[0], 2) + pow(vec[1], 2));
}

float distance(const Point2F& a, const Point2F& b)
{
  return len(getVec2F(a, b));
}

Vec2F devide(const Vec2F& vec, const float num)
{
  return { vec[0] / num, vec[1] / num };
}

Vec2F multiply(const Vec2F& vec, const float num)
{
  return { vec[0] * num, vec[1] * num };
}

Vec2F plus(const Vec2F& vecA, const Vec2F& vecB)
{
  return { vecA[0] + vecB[0], vecA[1] + vecB[1] };
}

Vec2F minus(const Vec2F& vecA, const Vec2F& vecB)
{
  return { vecA[0] - vecB[0], vecA[1] - vecB[1] };
}

struct OccupancyMap
{
  std::vector<std::vector<bool>> grids;
  Point2F topleftOrigin = { 0.f, 0.f };
  float gridSize = 0.01f;
  // the first grid including value at ([0, gridSize), [0, gridSize))
};

class RobotPlanner : public rclcpp::Node
{
public:
  RobotPlanner() : Node("robot_planner")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("status", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&RobotPlanner::timer_callback, this));
    service_ = this->create_service<cleaningbot_navigation_sim::srv::LoadPlanJson>(
        "load_plan_json", std::bind(&RobotPlanner::loadPlanJson, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    rclcpp::Time timestamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Message timestamp: %lf", timestamp.seconds());

    if (waypoints_.empty())
    {
      return;  // waiting for data inputs
    }
    // compute points, velocity
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! ";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
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
      robotContourPoints_ = jsonData["robot"];
      robotGadgetPoints_ = jsonData["cleaning_gadget"];
      for (const Point2F& point : jsonData["path"])
      {
        waypoints_.push_back(point);
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

    estimateTrajectory();
    RCLCPP_INFO(this->get_logger(), "Trajectory summary: distance=%f, duration=%f", distanceSum_, durationSum_);

    estimateMap();
    RCLCPP_INFO(this->get_logger(), "Map summary: num of occupied grids=%zu, covered area=%f", numOccGrids_, areaSum_);
  }

  void estimateTrajectory()
  {
    velocities_.reserve(waypoints_.size());
    velocities_[0] = velocityMin_;            // assume that initial velocity is velocityMin_
    velocities_[velocities_.size() - 1] = 0;  // assume that robot stops at the end
    for (std::size_t i = 1; i < waypoints_.size() - 1; i++)
    {
      const Point2F prev = waypoints_[i - 1];
      const Point2F cur = waypoints_[i];
      const Point2F next = waypoints_[i + 1];
      const Vec2F prevVec = { cur[0] - prev[0], cur[1] - prev[1] };
      const Vec2F nextVec = { next[0] - cur[0], next[1] - cur[1] };
      const float theta = acos(dot(prevVec, nextVec) / (len(prevVec) * len(nextVec)));
      const float avgLen = (len(prevVec) + len(nextVec)) / 2.f;
      const float curvature = theta / avgLen;
      velocities_[i] = std::isnan(curvature) ?
                           velocities_[i - 1] :
                           curvatureToVelocity(curvature);  // reuse prev vel if k is nan due to points too close
    }

    distances_.reserve(waypoints_.size() - 1);
    for (std::size_t i = 0; i < waypoints_.size() - 1; i++)
    {
      const Point2F cur = waypoints_[i];
      const Point2F next = waypoints_[i + 1];
      const Vec2F vecToNext = { next[0] - cur[0], next[1] - cur[1] };
      distances_[i] = len(vecToNext);
      distanceSum_ += distances_[i];
    }

    durations_.reserve(waypoints_.size() - 1);
    for (std::size_t i = 0; i < waypoints_.size() - 1; i++)
    {
      durations_[i] = distances_[i] / velocities_[i];
      durationSum_ += durations_[i];
    }
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

  void estimateMap()
  {
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

    const float gadgetLen = distance(robotGadgetPoints_[0], robotGadgetPoints_[1]);
    leftMost -= 0.5f * gadgetLen;
    topMost -= 0.5f * gadgetLen;
    rightMost += 0.5f * gadgetLen;
    bottomMost += 0.5f * gadgetLen;

    const int leftMostGridIdx = floor(leftMost / map.gridSize);
    const int topMostGridIdx = floor(topMost / map.gridSize);
    const int rightMostGridIdx = floor(rightMost / map.gridSize);
    const int bottomMostGridIdx = floor(bottomMost / map.gridSize);

    map.topleftOrigin[0] = leftMostGridIdx * map.gridSize;
    map.topleftOrigin[1] = topMostGridIdx * map.gridSize;
    const int width = rightMostGridIdx - leftMostGridIdx + 1;
    const int height = bottomMostGridIdx - topMostGridIdx + 1;
    map.grids = std::vector<std::vector<bool>>(height, std::vector<bool>(width, false));

    for (std::size_t i = 0; i < waypoints_.size() - 1; i++)
    {
      Vec2F verticalVec = getVec2F(waypoints_[i], waypoints_[i + 1]);
      Vec2F horizontalVec = { verticalVec[1], -verticalVec[0] };
      Vec2F horizontalUnitVec = devide(horizontalVec, len(horizontalVec));
      Vec2F horizontalUnitLeftVec =
          cross(verticalVec, horizontalUnitVec) > 0 ? horizontalUnitVec : multiply(horizontalUnitVec, -1.f);

      // get four points rectangle for the segment
      Point2F bottomleft = plus(waypoints_[i], multiply(horizontalUnitLeftVec, 0.5f * gadgetLen));
      Point2F bottomright = plus(waypoints_[i], multiply(horizontalUnitLeftVec, -0.5f * gadgetLen));
      Point2F topleft = plus(waypoints_[i + 1], multiply(horizontalUnitLeftVec, 0.5f * gadgetLen));
      Point2F topright = plus(waypoints_[i + 1], multiply(horizontalUnitLeftVec, -0.5f * gadgetLen));

      // it can then be four lines (vec)
      Vec2F rightVec = minus(topright, bottomright);
      Vec2F topVec = minus(topleft, topright);
      Vec2F lefVec = minus(bottomleft, topleft);
      Vec2F bottomtVec = minus(bottomright, bottomleft);

      // get bounding box for the rectangle
      float maxX = std::max(std::max(topleft[0], topright[0]), std::max(bottomleft[0], bottomright[0]));

      // iterate through all grids in bbox if the grid center located in four lines then mark as true. boundary cross
      // target >0 then inside cache the updated ids

      // get four points for footprint
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cleaningbot_navigation_sim::srv::LoadPlanJson>::SharedPtr service_;

  // inputs
  std::array<Point2F, 4> robotContourPoints_;
  std::array<Point2F, 2> robotGadgetPoints_;
  std::vector<Point2F> waypoints_;

  // data
  std::vector<float> velocities_;
  std::vector<float> distances_;
  std::vector<float> durations_;
  std::vector<std::array<Point2F, 4>> footprints_;
  std::vector<std::vector<Point2I>> updatedGridIds_;
  OccupancyMap map;

  // ans
  float distanceSum_ = 0.f;
  float durationSum_ = 0.f;
  float areaSum_ = 0.f;
  std::size_t numOccGrids_ = 0;

  // configs
  const float curvatureCritical_ = 0.5;
  const float curvatureMax_ = 10;
  const float velocityMin_ = 0.15;
  const float velocityMax_ = 1.1;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPlanner>());
  rclcpp::shutdown();
  return 0;
}