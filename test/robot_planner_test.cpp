#include "rclcpp/rclcpp.hpp"

#include "cleaningbot_navigation_sim/srv/load_plan_json.hpp"
#include "cleaningbot_navigation_sim/robot_planner.h"

#include <gtest/gtest.h>
#include <memory>
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

TEST_F(RobotPlannerTest, end2endTestLinearPath)
{
}

TEST_F(RobotPlannerTest, end2endTestCurvePath)
{
}
