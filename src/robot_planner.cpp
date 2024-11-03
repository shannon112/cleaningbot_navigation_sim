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

// Hermite basis functions
float h00(float t)
{
  return 2 * t * t * t - 3 * t * t + 1;
}
float h10(float t)
{
  return t * t * t - 2 * t * t + t;
}
float h01(float t)
{
  return -2 * t * t * t + 3 * t * t;
}
float h11(float t)
{
  return t * t * t - t * t;
}

// Function to interpolate between two points using Hermite cubic interpolation
Point2F cubicHermiteInterpolation(const Point2F& p0, const Point2F& p1, const Point2F& m0, const Point2F& m1, float t)
{
  float x = h00(t) * p0[0] + h10(t) * m0[0] + h01(t) * p1[0] + h11(t) * m1[0];
  float y = h00(t) * p0[1] + h10(t) * m0[1] + h01(t) * p1[1] + h11(t) * m1[1];
  return { x, y };
}

float dot(Vec2F vecA, Vec2F vecB)
{
  return vecA[0] * vecB[0] + vecA[1] * vecB[1];
}

float len(Vec2F vec)
{
  return sqrt(pow(vec[0], 2) + pow(vec[1], 2));
}

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

    // compute points, velocity
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
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

    estimateVelocities();
  }

  void estimateVelocities()
  {
    /*
    Point2F beginningWaypoint = {};
    Point2F endingWaypoint = {};
    // extrapolation
    Point2F p0 = {0.0f, 0.0f};  // Starting point
    Point2F p1 = {1.0f, 1.0f};  // Ending point
    Point2F m0 = {1.0f, 0.0f};  // Tangent at p0
    Point2F m1 = {1.0f, 1.0f};  // Tangent at p1
    float t = 0.5f; // Interpolation parameter between 0 and 1
    Point2F interpolatedPoint = cubicHermiteInterpolation(p0, p1, m0, m1, t);
    */

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

    RCLCPP_INFO(this->get_logger(), "distanceSum_ durationSum_ [%f, %f]", distanceSum_, durationSum_);
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

  int count_ = 0;
  bool isTrajectoryReady = false;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cleaningbot_navigation_sim::srv::LoadPlanJson>::SharedPtr service_;
  std::array<Point2F, 4> robotContourPoints_;
  std::array<Point2F, 2> robotGadgetPoints_;
  std::vector<Point2F> waypoints_;
  std::vector<float> velocities_;
  std::vector<float> distances_;
  float distanceSum_ = 0.f;
  std::vector<float> durations_;
  float durationSum_ = 0.f;

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