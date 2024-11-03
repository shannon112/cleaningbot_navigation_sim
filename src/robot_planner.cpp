#include "cleaningbot_navigation_sim/srv/load_plan_json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

class RobotPlanner : public rclcpp::Node
{
public:
  RobotPlanner() : Node("robot_planner")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("status", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&RobotPlanner::timer_callback, this));
    service_ = this->create_service<cleaningbot_navigation_sim::srv::LoadPlanJson>(
        "load_plan_json", std::bind(&RobotPlanner::load_plan_json, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // compute points, velocity
    publisher_->publish(message);
  }

  bool parse_plan_json(const std::string planJsonStr)
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
      for (const std::array<float, 2>& point : jsonData["path"])
      {
        waypoints_.push_back(point);
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

  void load_plan_json(const std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Request> request,
                      std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Loading... %s", request->plan_json.c_str());
    const bool isSuccessfullyLoaded = parse_plan_json(request->plan_json);

    response->is_successfully_loaded = isSuccessfullyLoaded;
    response->num_path_samples = waypoints_.size();
    RCLCPP_INFO(this->get_logger(), "Loading is %s, there are %u waypoints",
                response->is_successfully_loaded ? "success" : "failed", response->num_path_samples);
  }

  int count_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cleaningbot_navigation_sim::srv::LoadPlanJson>::SharedPtr service_;
  std::array<std::array<float, 2>, 4> robotContourPoints_;
  std::array<std::array<float, 2>, 2> robotGadgetPoints_;
  std::vector<std::array<float, 2>> waypoints_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPlanner>());
  rclcpp::shutdown();
  return 0;
}