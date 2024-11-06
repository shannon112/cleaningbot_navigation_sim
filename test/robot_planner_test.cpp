#include "rclcpp/rclcpp.hpp"

#include "cleaningbot_navigation_sim/robot_planner.h"

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <ament_index_cpp/get_package_prefix.hpp>

class RobotPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    robot_planner_ = std::make_shared<RobotPlanner>(nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<RobotPlanner> robot_planner_;
  std::string package_path_ = ament_index_cpp::get_package_prefix("cleaningbot_navigation_sim");
};

TEST_F(RobotPlannerTest, parsePlanJsonWithNoSuchFile)
{
  EXPECT_FALSE(robot_planner_->parsePlanJson("abc"));
}

TEST_F(RobotPlannerTest, parsePlanJsonWithoutEnoughPoints)
{
  EXPECT_FALSE(robot_planner_->parsePlanJson(package_path_ + "/test_vectors/one_point.json"));
  EXPECT_FALSE(robot_planner_->parsePlanJson(package_path_ + "/test_vectors/no_point.json"));
}

TEST_F(RobotPlannerTest, parsePlanJsonInvalidFormat)
{
  EXPECT_FALSE(robot_planner_->parsePlanJson(package_path_ + "/test_vectors/not_four_points_robot.json"));
  EXPECT_FALSE(robot_planner_->parsePlanJson(package_path_ + "/test_vectors/not_two_points_gadget.json"));
}

TEST_F(RobotPlannerTest, parsePlanJsonValid)
{
  EXPECT_TRUE(robot_planner_->parsePlanJson(package_path_ + "/test_vectors/two_points.json"));

  EXPECT_EQ(robot_planner_->getRobotContourPoints()[0][0], -1.4f);
  EXPECT_EQ(robot_planner_->getRobotContourPoints()[0][1], -1.3f);
  EXPECT_EQ(robot_planner_->getRobotContourPoints()[1][0], -1.4f);
  EXPECT_EQ(robot_planner_->getRobotContourPoints()[1][1], 1.3f);
  EXPECT_EQ(robot_planner_->getRobotContourPoints()[2][0], 1.4f);
  EXPECT_EQ(robot_planner_->getRobotContourPoints()[2][1], 1.3f);
  EXPECT_EQ(robot_planner_->getRobotContourPoints()[3][0], 1.4f);
  EXPECT_EQ(robot_planner_->getRobotContourPoints()[3][1], -1.3f);

  EXPECT_EQ(robot_planner_->getRobotGadgetPoints()[0][0], 0.4f);
  EXPECT_EQ(robot_planner_->getRobotGadgetPoints()[0][1], 0.5f);
  EXPECT_EQ(robot_planner_->getRobotGadgetPoints()[1][0], 0.4f);
  EXPECT_EQ(robot_planner_->getRobotGadgetPoints()[1][1], -0.5f);

  EXPECT_EQ(robot_planner_->getWaypoints()[0][0], 1.0f);
  EXPECT_EQ(robot_planner_->getWaypoints()[0][1], 0.0f);
  EXPECT_EQ(robot_planner_->getWaypoints()[1][0], -1.0f);
  EXPECT_EQ(robot_planner_->getWaypoints()[1][1], 0.0f);
}

class TestActionClient : public rclcpp::Node
{
public:
  using LoadPlanJson = cleaningbot_navigation_sim::action::LoadPlanJson;
  using GoalHandleLoadPlanJson = rclcpp_action::ClientGoalHandle<LoadPlanJson>;

  TestActionClient(const std::string& file) : Node("test_action_client")
  {
    this->client_ = rclcpp_action::create_client<LoadPlanJson>(this, "load_plan_json");
    this->send_goal(file);
  }

  void send_goal(const std::string& file)
  {
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = LoadPlanJson::Goal();
    goal_msg.plan_json = file;

    auto send_goal_options = rclcpp_action::Client<LoadPlanJson>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TestActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&TestActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&TestActionClient::result_callback, this, std::placeholders::_1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }
  float path_length_ = 0.f;
  float path_time_ = 0.f;
  float path_area_ = 0.f;

private:
  void goal_response_callback(std::shared_future<GoalHandleLoadPlanJson::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleLoadPlanJson::SharedPtr,
                         const std::shared_ptr<const LoadPlanJson::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "get feedback iteration %zu", feedback->iteration);
  }

  void result_callback(const GoalHandleLoadPlanJson::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    path_length_ = result.result->path_length;
    path_time_ = result.result->path_time;
    path_area_ = result.result->path_area;

    rclcpp::shutdown();
  }

  rclcpp_action::Client<LoadPlanJson>::SharedPtr client_;
};

TEST_F(RobotPlannerTest, end2endTestLinear0Path)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_planner_);
  std::atomic<bool> keep_running(true);
  std::thread spinThread([&executor, &keep_running]() {
    while (keep_running)
    {
      executor.spin_once(std::chrono::milliseconds(100));  // Spin periodically
    }
  });

  auto client = std::make_shared<TestActionClient>(package_path_ + "/test_vectors/line0.json");
  rclcpp::spin(client);
  EXPECT_NEAR(client->path_length_, 5.f, 10e-3);
  EXPECT_NEAR(client->path_time_, 5.f / 1.1f, 10e-3);
  EXPECT_NEAR(client->path_area_, 5.f, 10e-3);

  keep_running = false;
  spinThread.join();
  rclcpp::shutdown();
}

TEST_F(RobotPlannerTest, end2endTestLinear45Path)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_planner_);
  std::atomic<bool> keep_running(true);
  std::thread spinThread([&executor, &keep_running]() {
    while (keep_running)
    {
      executor.spin_once(std::chrono::milliseconds(100));  // Spin periodically
    }
  });

  auto client = std::make_shared<TestActionClient>(package_path_ + "/test_vectors/line45.json");
  rclcpp::spin(client);
  EXPECT_NEAR(client->path_length_, 5.f * sqrt(2), 10e-3);
  EXPECT_NEAR(client->path_time_, 5.f * sqrt(2) / 1.1f, 10e-3);
  EXPECT_NEAR(client->path_area_, 5.f * sqrt(2), 10e-1);

  keep_running = false;
  spinThread.join();
  rclcpp::shutdown();
}

TEST_F(RobotPlannerTest, end2endTestLinear90Path)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_planner_);
  std::atomic<bool> keep_running(true);
  std::thread spinThread([&executor, &keep_running]() {
    while (keep_running)
    {
      executor.spin_once(std::chrono::milliseconds(100));  // Spin periodically
    }
  });

  auto client = std::make_shared<TestActionClient>(package_path_ + "/test_vectors/line90.json");
  rclcpp::spin(client);
  EXPECT_NEAR(client->path_length_, 5.f, 10e-3);
  EXPECT_NEAR(client->path_time_, 5.f / 1.1f, 10e-3);
  EXPECT_NEAR(client->path_area_, 5.f, 10e-3);

  keep_running = false;
  spinThread.join();
  rclcpp::shutdown();
}

TEST_F(RobotPlannerTest, end2endTestCurvePath)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_planner_);
  std::atomic<bool> keep_running(true);
  std::thread spinThread([&executor, &keep_running]() {
    while (keep_running)
    {
      executor.spin_once(std::chrono::milliseconds(100));  // Spin periodically
    }
  });

  auto client = std::make_shared<TestActionClient>(package_path_ + "/test_vectors/circle.json");
  rclcpp::spin(client);
  EXPECT_NEAR(client->path_length_, 2.f * M_PI * 2.f, 10e-3);
  EXPECT_NEAR(client->path_time_, 2.f * M_PI * 2.f / 1.1f, 10e-1);
  EXPECT_NEAR(client->path_area_, std::pow(2.5f, 2) * M_PI - std::pow(1.5f, 2) * M_PI, 1.f);

  keep_running = false;
  spinThread.join();
  rclcpp::shutdown();
}
