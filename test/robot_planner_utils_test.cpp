#include <gtest/gtest.h>
#include "cleaningbot_navigation_sim/robot_planner_utils.hpp"

TEST(RobotPlannerUtilsTest, getRotationMat)
{
  EXPECT_NEAR(0, getRotationMat(Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f)).angle(), 0.0001f);
  EXPECT_NEAR(0.5f * M_PI, getRotationMat(Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 1.f)).angle(), 0.0001f);
  EXPECT_NEAR(M_PI, getRotationMat(Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(-1.f, 0.f)).angle(), 0.0001f);
  EXPECT_NEAR(-0.5f * M_PI, getRotationMat(Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, -1.f)).angle(), 0.0001f);
}

TEST(RobotPlannerUtilsTest, constructMapWithoutGridSize)
{
  const std::array<Eigen::Vector2f, 4> robotContourPoints = {};
  const std::array<Eigen::Vector2f, 2>& robotGadgetPoints = {};
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 0.f) };
  const float gridSize = 0.f;
  OccupancyMap map = constructMap(robotContourPoints, robotGadgetPoints, waypoints, gridSize);
  EXPECT_EQ(map.gridSize, 0.f);
  EXPECT_EQ(map.origin[0], 0.f);
  EXPECT_EQ(map.origin[1], 0.f);
  EXPECT_EQ(map.grids.rows(), 0);
  EXPECT_EQ(map.grids.cols(), 0);
}

TEST(RobotPlannerUtilsTest, constructMapWithoutPoints)
{
  const std::array<Eigen::Vector2f, 4> robotContourPoints = {};
  const std::array<Eigen::Vector2f, 2>& robotGadgetPoints = {};
  const std::vector<Eigen::Vector2f> waypoints = {};
  const float gridSize = 1.f;
  OccupancyMap map = constructMap(robotContourPoints, robotGadgetPoints, waypoints, gridSize);
  EXPECT_EQ(map.gridSize, 0.f);
  EXPECT_EQ(map.origin[0], 0.f);
  EXPECT_EQ(map.origin[1], 0.f);
  EXPECT_EQ(map.grids.rows(), 0);
  EXPECT_EQ(map.grids.cols(), 0);
}

TEST(RobotPlannerUtilsTest, constructMapWithPaddingForRobot)
{
  const std::array<Eigen::Vector2f, 4> robotContourPoints = { Eigen::Vector2f(-0.5f, -0.5f),
                                                              Eigen::Vector2f(-0.5f, 0.5f), Eigen::Vector2f(0.5f, 0.5f),
                                                              Eigen::Vector2f(0.5f, -0.5f) };
  const std::array<Eigen::Vector2f, 2>& robotGadgetPoints = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 0.f) };
  const float gridSize = 1.f;
  OccupancyMap map = constructMap(robotContourPoints, robotGadgetPoints, waypoints, gridSize);
  EXPECT_EQ(map.gridSize, 1.f);
  EXPECT_EQ(map.origin[0], -1.f);
  EXPECT_EQ(map.origin[1], -1.f);
  EXPECT_EQ(map.grids.rows(), 2);
  EXPECT_EQ(map.grids.cols(), 2);
}

TEST(RobotPlannerUtilsTest, constructMapWithPaddingForGadget)
{
  const std::array<Eigen::Vector2f, 4> robotContourPoints = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 0.f),
                                                              Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 0.f) };
  const std::array<Eigen::Vector2f, 2>& robotGadgetPointsInsideGrid = { Eigen::Vector2f(0.f, 1.99f),
                                                                        Eigen::Vector2f(0.f, -1.99f) };
  const std::array<Eigen::Vector2f, 2>& robotGadgetPointsOnGrid = { Eigen::Vector2f(0.f, 2.f),
                                                                    Eigen::Vector2f(0.f, -2.f) };
  const std::array<Eigen::Vector2f, 2>& robotGadgetPointsOutsideGrid = { Eigen::Vector2f(0.f, 2.01f),
                                                                         Eigen::Vector2f(0.f, -2.01f) };
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 0.f) };
  const float gridSize = 1.f;
  OccupancyMap map = constructMap(robotContourPoints, robotGadgetPointsInsideGrid, waypoints, gridSize);
  EXPECT_EQ(map.gridSize, 1.f);
  EXPECT_EQ(map.origin[0], -2.0f);
  EXPECT_EQ(map.origin[1], -2.0f);
  EXPECT_EQ(map.grids.rows(), 4);
  EXPECT_EQ(map.grids.cols(), 4);

  map = constructMap(robotContourPoints, robotGadgetPointsOnGrid, waypoints, gridSize);
  EXPECT_EQ(map.origin[0], -2.0f);
  EXPECT_EQ(map.origin[1], -2.0f);
  EXPECT_EQ(map.grids.rows(), 5);
  EXPECT_EQ(map.grids.cols(), 5);

  map = constructMap(robotContourPoints, robotGadgetPointsOutsideGrid, waypoints, gridSize);
  EXPECT_EQ(map.origin[0], -3.0f);
  EXPECT_EQ(map.origin[1], -3.0f);
  EXPECT_EQ(map.grids.rows(), 6);
  EXPECT_EQ(map.grids.cols(), 6);
}

TEST(RobotPlannerUtilsTest, constructMapWithoutPadding)
{
}

TEST(RobotPlannerUtilsTest, constructMap)
{
}

TEST(RobotPlannerUtilsTest, simplifyTrajectoryReturnEmpty)
{
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypointsEmpty = {};

  EXPECT_TRUE(waypointsEmpty == simplifyTrajectory(waypointsEmpty, 0.01f));
  EXPECT_TRUE(waypointsEmpty == simplifyTrajectory(waypoints, -0.01f));
  EXPECT_TRUE(waypointsEmpty == simplifyTrajectory(waypoints, 0.0f));
}

TEST(RobotPlannerUtilsTest, simplifyTrajectoryWithOnePoint)
{
  const std::vector<Eigen::Vector2f> waypointOne = { Eigen::Vector2f(0.f, 0.f) };
  EXPECT_TRUE(waypointOne == simplifyTrajectory(waypointOne, 0.1123f));
  EXPECT_TRUE(waypointOne == simplifyTrajectory(waypointOne, 12323.11f));
}

TEST(RobotPlannerUtilsTest, simplifyTrajectoryShort)
{
  const std::vector<Eigen::Vector2f> waypointOne = { Eigen::Vector2f(0.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 0.11f) };
  EXPECT_TRUE(waypointOne == simplifyTrajectory(waypoints, 0.12f));
  EXPECT_TRUE(waypoints == simplifyTrajectory(waypoints, 0.11f));
  EXPECT_TRUE(waypoints == simplifyTrajectory(waypoints, 0.10f));
}

TEST(RobotPlannerUtilsTest, simplifyTrajectoryLong)
{
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 0.1f),
                                                   Eigen::Vector2f(0.f, 0.2f), Eigen::Vector2f(0.f, 0.3f),
                                                   Eigen::Vector2f(0.f, 0.4f) };
  const std::vector<Eigen::Vector2f> waypointsSim = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 0.2f),
                                                      Eigen::Vector2f(0.f, 0.4f) };
  EXPECT_TRUE(waypointsSim == simplifyTrajectory(waypoints, 0.2f));
}

TEST(RobotPlannerUtilsTest, resampleTrajectoryReturnEmpty)
{
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypointsEmpty = {};

  EXPECT_TRUE(waypointsEmpty == resampleTrajectory(waypointsEmpty, 2));
  EXPECT_TRUE(waypointsEmpty == resampleTrajectory(waypoints, 0));
  EXPECT_TRUE(waypointsEmpty == resampleTrajectory(waypoints, 1));
}

TEST(RobotPlannerUtilsTest, resampleTrajectoryWithOnePoint)
{
  const std::vector<Eigen::Vector2f> waypointOne = { Eigen::Vector2f(0.f, 0.f) };
  EXPECT_TRUE(waypointOne == resampleTrajectory(waypointOne, 2));
  EXPECT_TRUE(waypointOne == resampleTrajectory(waypointOne, 12312));
}

TEST(RobotPlannerUtilsTest, resampleTrajectoryLinear)
{
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(4.0f, 0.0f) };
  const std::size_t trajectoryUpSamplingNumIntervals = 4;
  std::vector<Eigen::Vector2f> expected = { Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 0.0f),
                                            Eigen::Vector2f(2.0f, 0.0f), Eigen::Vector2f(3.0f, 0.0f),
                                            Eigen::Vector2f(4.0f, 0.0f) };
  EXPECT_TRUE(expected == resampleTrajectory(waypoints, trajectoryUpSamplingNumIntervals));
}

TEST(RobotPlannerUtilsTest, resampleTrajectoryCubic)
{
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(2.0f, 2.0f),
                                                   Eigen::Vector2f(4.0f, 1.0f), Eigen::Vector2f(6.0f, 3.0f) };
  const std::size_t trajectoryUpSamplingNumIntervals = 2;
  const std::vector<Eigen::Vector2f> expected = { Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(1.0f, 1.1875f),
                                                  Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(3.0f, 1.5f),
                                                  Eigen::Vector2f(4.0f, 1.0f), Eigen::Vector2f(5.0f, 1.8125f),
                                                  Eigen::Vector2f(6.0f, 3.0f) };
  EXPECT_TRUE(expected == resampleTrajectory(waypoints, trajectoryUpSamplingNumIntervals));
}

TEST(RobotPlannerUtilsTest, curvatureToVelocity)
{
  const float curvatureCritical = 10.0f;
  const float curvatureMax = 20.f;
  const float velocityMin = 1.0f;
  const float velocityMax = 2.0f;
  EXPECT_EQ(velocityMin, curvatureToVelocity(curvatureMax, curvatureCritical, curvatureMax, velocityMin, velocityMax));
  EXPECT_EQ(velocityMax - 0.7f, curvatureToVelocity(17.f, curvatureCritical, curvatureMax, velocityMin, velocityMax));
  EXPECT_EQ(velocityMax - 0.5f, curvatureToVelocity(15.f, curvatureCritical, curvatureMax, velocityMin, velocityMax));
  EXPECT_EQ(velocityMax - 0.2f, curvatureToVelocity(12.f, curvatureCritical, curvatureMax, velocityMin, velocityMax));
  EXPECT_EQ(velocityMax,
            curvatureToVelocity(curvatureCritical, curvatureCritical, curvatureMax, velocityMin, velocityMax));
}

TEST(RobotPlannerUtilsTest, estimateVelocityBeginMidEnd)
{
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(2.0f, 2.0f),
                                                   Eigen::Vector2f(4.0f, 0.0f) };
  const float curvatureApprxDist = 1.f;
  const float curvatureCritical = 0.5f;
  const float curvatureMax = 10.f;
  const float velocityMin = 1.0f;
  const float velocityMax = 2.0f;
  const float velocityPrev = 1.234f;
  EXPECT_EQ(velocityMax, estimateVelocity(waypoints, 0, curvatureApprxDist, curvatureCritical, curvatureMax,
                                          velocityMin, velocityMax, velocityPrev));
  EXPECT_GT(velocityMax, estimateVelocity(waypoints, 1, curvatureApprxDist, curvatureCritical, curvatureMax,
                                          velocityMin, velocityMax, velocityPrev));
  EXPECT_LT(velocityMin, estimateVelocity(waypoints, 1, curvatureApprxDist, curvatureCritical, curvatureMax,
                                          velocityMin, velocityMax, velocityPrev));
  EXPECT_EQ(0, estimateVelocity(waypoints, 2, curvatureApprxDist, curvatureCritical, curvatureMax, velocityMin,
                                velocityMax, velocityPrev));
}

TEST(RobotPlannerUtilsTest, estimateVelocityNoEnoughLength)
{
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 0.f),
                                                   Eigen::Vector2f(0.f, 0.f) };
  const float curvatureApprxDist = 0.1f;
  const float curvatureCritical = 0.5f;
  const float curvatureMax = 10.f;
  const float velocityMin = 1.0f;
  const float velocityMax = 2.0f;
  const float velocityPrev = 1.234f;
  EXPECT_EQ(velocityPrev, estimateVelocity(waypoints, 1, curvatureApprxDist, curvatureCritical, curvatureMax,
                                           velocityMin, velocityMax, velocityPrev));
}

TEST(RobotPlannerUtilsTest, estimateVelocityApproximation)
{
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(2.0f, 2.0f),
                                                   Eigen::Vector2f(2.0f, 2.0f), Eigen::Vector2f(2.0f, 2.0f),
                                                   Eigen::Vector2f(4.0f, 0.0f) };
  const float curvatureApprxDist = 1.f;
  const float curvatureCritical = 0.5f;
  const float curvatureMax = 10.f;
  const float velocityMin = 1.0f;
  const float velocityMax = 2.0f;
  const float velocityPrev = 1.234f;
  const float v1 = estimateVelocity(waypoints, 1, curvatureApprxDist, curvatureCritical, curvatureMax, velocityMin,
                                    velocityMax, velocityPrev);
  const float v2 = estimateVelocity(waypoints, 2, curvatureApprxDist, curvatureCritical, curvatureMax, velocityMin,
                                    velocityMax, velocityPrev);
  const float v3 = estimateVelocity(waypoints, 3, curvatureApprxDist, curvatureCritical, curvatureMax, velocityMin,
                                    velocityMax, velocityPrev);
  EXPECT_TRUE(velocityMin < v1 && v1 < velocityMax);
  EXPECT_TRUE(v1 != velocityPrev);
  EXPECT_TRUE(v1 == v2 && v2 == v3);
}

TEST(RobotPlannerUtilsTest, estimateNewCoveredGridIdsToNext)
{
}

TEST(RobotPlannerUtilsTest, estimateTransformedPointsAtBeginning)
{
  const std::size_t curIdx = 0;
  const std::array<Eigen::Vector2f, 2> shape = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints0 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints90 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 1.f) };

  const std::array<Eigen::Vector2f, 2> shape0 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  const std::array<Eigen::Vector2f, 2> shape90 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 1.f) };
  EXPECT_TRUE(shape0 == estimateTransformedPoints(shape, waypoints0, curIdx));
  EXPECT_TRUE(shape90 == estimateTransformedPoints(shape, waypoints90, curIdx));  // near
}
