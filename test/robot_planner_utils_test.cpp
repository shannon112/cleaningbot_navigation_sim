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

TEST(RobotPlannerUtilsTest, simplifyTrajectoryInvaid)
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

TEST(RobotPlannerUtilsTest, resampleTrajectoryInvaid)
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

TEST(RobotPlannerUtilsTest, estimateNewCoveredGridIdsToNextAtTheEnd)
{
  const int width = 5;
  const int height = 5;
  OccupancyMap map;
  map.grids = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(width, height);
  map.origin = { 0.f, 0.f };
  map.gridSize = 1.f;
  const std::array<Eigen::Vector2f, 2> robotGadgetPoints = { Eigen::Vector2f(0.f, 0.5f), Eigen::Vector2f(0.f, -0.5f) };
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 3.f), Eigen::Vector2f(1.f, 3.f) };

  EXPECT_TRUE(estimateNewCoveredGridIdsToNext(map, waypoints, 1, robotGadgetPoints).empty());
  EXPECT_FALSE(estimateNewCoveredGridIdsToNext(map, waypoints, 0, robotGadgetPoints).empty());
}

TEST(RobotPlannerUtilsTest, estimateNewCoveredGridIdsToNextWithoutPath)
{
  const int width = 5;
  const int height = 5;
  OccupancyMap map;
  map.grids = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(width, height);
  map.origin = { 0.f, 0.f };
  map.gridSize = 1.f;
  const std::array<Eigen::Vector2f, 2> robotGadgetPoints = { Eigen::Vector2f(0.f, 0.5f), Eigen::Vector2f(0.f, -0.5f) };

  const std::vector<Eigen::Vector2f> twoWaypoints = { Eigen::Vector2f(0.f, 3.f), Eigen::Vector2f(1.f, 3.f) };
  const std::vector<Eigen::Vector2f> oneWaypoints = { Eigen::Vector2f(1.f, 3.f) };
  const std::vector<Eigen::Vector2f> noWaypoints = {};

  EXPECT_TRUE(estimateNewCoveredGridIdsToNext(map, oneWaypoints, 0, robotGadgetPoints).empty());
  EXPECT_TRUE(estimateNewCoveredGridIdsToNext(map, noWaypoints, 0, robotGadgetPoints).empty());
  EXPECT_TRUE(estimateNewCoveredGridIdsToNext(map, twoWaypoints, 2, robotGadgetPoints).empty());
  EXPECT_FALSE(estimateNewCoveredGridIdsToNext(map, twoWaypoints, 0, robotGadgetPoints).empty());
}

TEST(RobotPlannerUtilsTest, estimateNewCoveredGridIdsToNextWithoutGadget)
{
  const int width = 5;
  const int height = 5;
  OccupancyMap map;
  map.grids = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(width, height);
  map.origin = { 0.f, 0.f };
  map.gridSize = 1.f;
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 3.f), Eigen::Vector2f(1.f, 3.f) };
  const std::size_t curIdx = 0;

  const std::array<Eigen::Vector2f, 2> robotGadgetPointsNo = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 0.f) };
  EXPECT_TRUE(estimateNewCoveredGridIdsToNext(map, waypoints, curIdx, robotGadgetPointsNo).empty());
  const std::array<Eigen::Vector2f, 2> robotGadgetPoints = { Eigen::Vector2f(0.f, 0.5f), Eigen::Vector2f(0.f, -0.5f) };
  EXPECT_FALSE(estimateNewCoveredGridIdsToNext(map, waypoints, curIdx, robotGadgetPoints).empty());
}

TEST(RobotPlannerUtilsTest, estimateNewCoveredGridIdsToNextWithoutMap)
{
  OccupancyMap map;
  map.origin = { 0.f, 0.f };
  const std::vector<Eigen::Vector2f> waypoints = { Eigen::Vector2f(0.f, 3.f), Eigen::Vector2f(1.f, 3.f) };
  const std::array<Eigen::Vector2f, 2> robotGadgetPoints = { Eigen::Vector2f(0.f, 0.5f), Eigen::Vector2f(0.f, -0.5f) };
  const std::size_t curIdx = 0;

  map.gridSize = 1.f;
  EXPECT_TRUE(estimateNewCoveredGridIdsToNext(map, waypoints, curIdx, robotGadgetPoints).empty());
  const int width = 5;
  const int height = 5;
  map.grids = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(width, height);
  EXPECT_FALSE(estimateNewCoveredGridIdsToNext(map, waypoints, curIdx, robotGadgetPoints).empty());
  map.gridSize = 0.f;
  EXPECT_TRUE(estimateNewCoveredGridIdsToNext(map, waypoints, curIdx, robotGadgetPoints).empty());
  map.gridSize = 1.f;
  EXPECT_FALSE(estimateNewCoveredGridIdsToNext(map, waypoints, curIdx, robotGadgetPoints).empty());
}

TEST(RobotPlannerUtilsTest, estimateNewCoveredGridIdsToNextToyCases)
{
  const int width = 5;
  const int height = 5;
  OccupancyMap map;
  map.grids = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(width, height);
  map.origin = { 0.f, 0.f };
  map.gridSize = 1.f;
  const std::size_t curIdx = 0;
  const std::array<Eigen::Vector2f, 2> robotGadgetPoints = { Eigen::Vector2f(0.f, 0.5f), Eigen::Vector2f(0.f, -0.5f) };

  // step on grid to cover
  const std::vector<Eigen::Vector2f> waypointsToGrid = { Eigen::Vector2f(0.f, 3.f), Eigen::Vector2f(1.f, 3.f) };
  const std::vector<Eigen::Vector2i> expectedCovered2 = { Eigen::Vector2i(0, 2), Eigen::Vector2i(0, 3) };
  EXPECT_TRUE(expectedCovered2 == estimateNewCoveredGridIdsToNext(map, waypointsToGrid, curIdx, robotGadgetPoints));

  // step on center to cover
  const std::vector<Eigen::Vector2f> waypointsToCenter = { Eigen::Vector2f(0.f, 3.f), Eigen::Vector2f(1.5f, 3.f) };
  const std::vector<Eigen::Vector2i> expectedCovered4 = { Eigen::Vector2i(0, 2), Eigen::Vector2i(0, 3),
                                                          Eigen::Vector2i(1, 2), Eigen::Vector2i(1, 3) };
  EXPECT_TRUE(expectedCovered4 == estimateNewCoveredGridIdsToNext(map, waypointsToCenter, curIdx, robotGadgetPoints));

  // step is too small to cover
  const std::vector<Eigen::Vector2f> waypointsSmall = { Eigen::Vector2f(0.f, 3.f), Eigen::Vector2f(0.49f, 3.f) };
  const std::vector<Eigen::Vector2i> expectedCovered0 = {};
  EXPECT_TRUE(expectedCovered0 == estimateNewCoveredGridIdsToNext(map, waypointsSmall, curIdx, robotGadgetPoints));
}

bool compareVectors(const std::array<Eigen::Vector2f, 2>& vec1, const std::array<Eigen::Vector2f, 2>& vec2,
                    float tolerance = 1e-5)
{
  if (vec1.size() != vec2.size())
    return false;
  for (size_t i = 0; i < vec1.size(); ++i)
  {
    if ((vec1[i] - vec2[i]).norm() > tolerance)
      return false;
  }
  return true;
}

TEST(RobotPlannerUtilsTest, estimateTransformedPointsInvalid)
{
  const std::array<Eigen::Vector2f, 2> shape = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints0 = {};
  const std::vector<Eigen::Vector2f> waypoints1 = { Eigen::Vector2f(0.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints2 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  EXPECT_TRUE(compareVectors(shape, estimateTransformedPoints(shape, waypoints0, 0)));
  EXPECT_TRUE(compareVectors(shape, estimateTransformedPoints(shape, waypoints1, 0)));
  EXPECT_TRUE(compareVectors(shape, estimateTransformedPoints(shape, waypoints2, 2)));
}

TEST(RobotPlannerUtilsTest, estimateTransformedPointsAtTheBeginningValid)
{
  const std::size_t curIdx = 0;
  const std::array<Eigen::Vector2f, 2> shape = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints0 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints90 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 1.f) };
  const std::vector<Eigen::Vector2f> waypoints180 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(-1.f, 0.f) };
  const std::vector<Eigen::Vector2f> waypoints270 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, -1.f) };

  const std::array<Eigen::Vector2f, 2> expectedShape0 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  const std::array<Eigen::Vector2f, 2> expectedShape90 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 1.f) };
  const std::array<Eigen::Vector2f, 2> expectedShape180 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(-1.f, 0.f) };
  const std::array<Eigen::Vector2f, 2> expectedShape270 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, -1.f) };

  const std::array<Eigen::Vector2f, 2> estShape0 = estimateTransformedPoints(shape, waypoints0, curIdx);
  const std::array<Eigen::Vector2f, 2> estShape90 = estimateTransformedPoints(shape, waypoints90, curIdx);
  const std::array<Eigen::Vector2f, 2> estShape180 = estimateTransformedPoints(shape, waypoints180, curIdx);
  const std::array<Eigen::Vector2f, 2> estShape270 = estimateTransformedPoints(shape, waypoints270, curIdx);

  EXPECT_TRUE(compareVectors(expectedShape0, estShape0));
  EXPECT_TRUE(compareVectors(expectedShape90, estShape90));
  EXPECT_TRUE(compareVectors(expectedShape180, estShape180));
  EXPECT_TRUE(compareVectors(expectedShape270, estShape270));
}

TEST(RobotPlannerUtilsTest, estimateTransformedPointsAtTheEndValid)
{
  const std::array<Eigen::Vector2f, 2> shape = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(1.f, 0.f) };
  const std::array<Eigen::Vector2f, 2> expectedShape = { Eigen::Vector2f(0.f, 1.f), Eigen::Vector2f(0.f, 2.f) };
  const std::vector<Eigen::Vector2f> waypoints2 = { Eigen::Vector2f(0.f, 0.f), Eigen::Vector2f(0.f, 1.f) };
  EXPECT_TRUE(compareVectors(expectedShape, estimateTransformedPoints(shape, waypoints2, 1)));
}