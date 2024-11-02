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
    publisher_->publish(message);
  }

  void load_plan_json(const std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Request> request,
                      std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Response> response)
  {
    const std::string planJsonStr = request->plan_json;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading... %s", request->plan_json.c_str());

    std::ifstream file(planJsonStr);
    if (!file.is_open())
    {
      response->is_successfully_loaded = false;
      response->num_path_samples = 0;
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open file");
      return;
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
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "JSON parsing error: %s", e.what());
    }

    std::cout << robotContourPoints_[0][0] << robotContourPoints_[0][1] << std::endl;
    std::cout << robotGadgetPoints_[0][0] << robotGadgetPoints_[0][1] << std::endl;
    std::cout << waypoints_[0][0] << waypoints_[0][1] << std::endl;

    response->is_successfully_loaded = true;
    response->num_path_samples = waypoints_.size();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading is %s, there are %u waypoints",
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