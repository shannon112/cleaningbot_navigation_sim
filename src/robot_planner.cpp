#include "rclcpp/rclcpp.hpp"
#include "cleaningbot_navigation_sim/srv/load_plan_json.hpp"

#include <memory>

void LoadPlanJson(const std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Request> request,
                         std::shared_ptr<cleaningbot_navigation_sim::srv::LoadPlanJson::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Load %s", request->plan_json);

  response->is_successfully_loaded = true;
  response->num_path_samples = 0;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading is %s, there are %u waypoints", response->is_successfully_loaded ? "success" : "failed", response->num_path_samples);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("load_plan_json_server");

  rclcpp::Service<cleaningbot_navigation_sim::srv::LoadPlanJson>::SharedPtr service =
    node->create_service<cleaningbot_navigation_sim::srv::LoadPlanJson>("load_plan_json", &LoadPlanJson);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}