#ifndef ROBOT_PLANNER_UTILS_H
#define ROBOT_PLANNER_UTILS_H

#include "cleaningbot_navigation_sim/occupancy_map.h"

#include <Eigen/Dense>

inline float cross2d(const Eigen::Vector2f& vecA, const Eigen::Vector2f& vecB)
{
  return vecA[0] * vecB[1] - vecA[1] * vecB[0];
}

inline Eigen::Rotation2D<float> getRotationMat(const Eigen::Vector2f& vecCur, const Eigen::Vector2f& vecNext)
{
  const Eigen::Vector2f translationVec = vecNext - vecCur;
  return Eigen::Rotation2D<float>(std::atan2(translationVec[1], translationVec[0]));
}

// create a plain dense map according to bounding box of the trajactory and the robot
inline OccupancyMap constructMap(const std::array<Eigen::Vector2f, 4>& robotContourPoints,
                                 const std::array<Eigen::Vector2f, 2>& robotGadgetPoints,
                                 const std::vector<Eigen::Vector2f>& waypoints, const float mapGridSize_)
{
  OccupancyMap map;
  map.gridSize = mapGridSize_;

  float leftMost = std::numeric_limits<float>::max();
  float rightMost = std::numeric_limits<float>::min();
  float topMost = std::numeric_limits<float>::min();
  float bottomMost = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < waypoints.size(); i++)
  {
    leftMost = std::min(leftMost, waypoints[i][0]);
    rightMost = std::max(rightMost, waypoints[i][0]);
    topMost = std::max(topMost, waypoints[i][1]);
    bottomMost = std::min(bottomMost, waypoints[i][1]);
  }

  const std::vector<float> robotPointLens = { robotGadgetPoints[0].norm(),  robotGadgetPoints[1].norm(),
                                              robotContourPoints[0].norm(), robotContourPoints[1].norm(),
                                              robotContourPoints[2].norm(), robotContourPoints[3].norm() };
  const float linkMaxLen = *std::max_element(robotPointLens.begin(), robotPointLens.end());
  const Eigen::Vector2f bottomleftMostPoint(leftMost - linkMaxLen, bottomMost - linkMaxLen);
  const Eigen::Vector2f topRightMostPoint(rightMost + linkMaxLen, topMost + linkMaxLen);

  const Eigen::Vector2i bottomleftMostGridIdx = (bottomleftMostPoint / map.gridSize).cast<int>();
  const Eigen::Vector2i topRightGridIdx = (topRightMostPoint / map.gridSize).cast<int>();
  map.origin = bottomleftMostGridIdx.cast<float>() * map.gridSize;

  const Eigen::Vector2i heightWidth = topRightGridIdx - bottomleftMostGridIdx + Eigen::Vector2i(1, 1);
  map.grids = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(heightWidth[1], heightWidth[0]);
  return map;
}

// downsample the trajectory by extracting key waypoints if there are sufficient offsets
inline std::vector<Eigen::Vector2f> simplifyTrajectory(const std::vector<Eigen::Vector2f>& waypoints,
                                                       const float trajectoryDownSamplingDist)
{
  if (waypoints.empty())
    return {};
  std::vector<Eigen::Vector2f> waypointsSim = { waypoints[0] };
  for (std::size_t i = 1; i < waypoints.size(); i++)
  {
    if ((waypoints[i] - waypointsSim[waypointsSim.size() - 1]).norm() >= trajectoryDownSamplingDist)
    {
      waypointsSim.push_back(waypoints[i]);
    }
  }
  return waypointsSim;
}

// upsample the trajectory using cubic interpolation
inline std::vector<Eigen::Vector2f> resampleTrajectory(const std::vector<Eigen::Vector2f>& waypoints,
                                                       const float trajectoryUpSamplingDist)
{
  if (waypoints.empty())
    return {};
  std::vector<Eigen::Vector2f> waypointsRe = {};
  for (std::size_t i = 0; i < waypoints.size() - 1; i++)
  {
    waypointsRe.push_back(waypoints[i]);
    const std::size_t numSamples = ((waypoints[i] - waypoints[i + 1]).norm()) / trajectoryUpSamplingDist;
    for (std::size_t j = 0; j < numSamples - 1; j++)
    {
      float t = (j + 1.f) / numSamples;
      // cubic Hermite spline interpolation
      const Eigen::Vector2f p0 = waypoints[i];
      const Eigen::Vector2f p1 = waypoints[i + 1];
      const Eigen::Vector2f m0 =
          (i == 0) ? (waypoints[1] - waypoints[0]) * 1.f : (waypoints[i + 1] - waypoints[i - 1]) * 0.5f;
      const Eigen::Vector2f m1 = (i == waypoints.size() - 2) ?
                                     (waypoints[waypoints.size() - 1] - waypoints[waypoints.size() - 2]) * 1.f :
                                     (waypoints[i + 2] - waypoints[i]) * 0.5f;

      const float t2 = t * t;
      const float t3 = t2 * t;

      const Eigen::Vector2f h00 = (2 * t3 - 3 * t2 + 1) * p0;  // basis function for p0
      const Eigen::Vector2f h10 = (t3 - 2 * t2 + t) * m0;      // basis function for m0
      const Eigen::Vector2f h01 = (-2 * t3 + 3 * t2) * p1;     // basis function for p1
      const Eigen::Vector2f h11 = (t3 - t2) * m1;              // basis function for m1
      waypointsRe.push_back(h00 + h10 + h01 + h11);

      // linear iterpolation
      // waypointsRe.push_back(waypoints[i] + t * (waypoints[i + 1] - waypoints[i]));
    }
  }
  waypointsRe.push_back(waypoints[waypoints.size() - 1]);
  return waypointsRe;
}

// velocity is proportional to the absolute curvature
inline float curvatureToVelocity(const float curvature, const float curvatureCritical, const float curvatureMax,
                                 const float velocityMin, const float velocityMax)
{
  if (curvature < curvatureCritical)
  {
    return velocityMax;
  }
  else if (curvatureMax <= curvature)
  {
    return velocityMin;
  }
  else
  {
    return velocityMax -
           (velocityMax - velocityMin) * (curvature - curvatureCritical) / (curvatureMax - curvatureCritical);
  }
}

// estimate velocity using approximated curvature
inline float estimateVelocity(const std::vector<Eigen::Vector2f>& waypoints, const std::size_t curIdx,
                              const float curvatureApprxDist, const float curvatureCritical, const float curvatureMax,
                              const float velocityMin, const float velocityMax, const float velocityPrev)
{
  assert(waypoints.size() > curIdx);
  assert(waypoints.size() > 0);
  if (curIdx == 0)
  {
    return velocityMax;  // assume that initial velocity is velocityMax
  }
  else if (curIdx == waypoints.size() - 1)
  {
    return 0.f;  // assume that robot stops at the end
  }
  else
  {
    std::size_t prevIdx = curIdx - 1;
    while (prevIdx != 0 && (waypoints[curIdx] - waypoints[prevIdx]).norm() < curvatureApprxDist)
      prevIdx--;
    const Eigen::Vector2f prevVec = waypoints[curIdx] - waypoints[prevIdx];

    std::size_t nextIdx = curIdx + 1;
    while (nextIdx != waypoints.size() - 1 && (waypoints[curIdx] - waypoints[nextIdx]).norm() < curvatureApprxDist)
      nextIdx++;
    const Eigen::Vector2f nextVec = waypoints[nextIdx] - waypoints[curIdx];

    const float theta = acos(prevVec.dot(nextVec) / (prevVec.norm() * nextVec.norm()));
    const float avgLen = (prevVec.norm() + nextVec.norm()) / 2.f;
    const float curvature = theta / avgLen;  // approximation
    return std::isnan(curvature) ?
               velocityPrev :
               curvatureToVelocity(curvature, curvatureCritical, curvatureMax, velocityMin,
                                   velocityMax);  // reuse prev vel if k is nan due to points too close
  }
}

// estimate the grids inside the covered area using cross product
std::vector<Eigen::Vector2i> estimateNewCoveredGridIdsToNext(const OccupancyMap& map,
                                                             const std::vector<Eigen::Vector2f>& waypoints,
                                                             const std::size_t curIdx,
                                                             const std::array<Eigen::Vector2f, 2>& robotGadgetPoints)
{
  if (curIdx == waypoints.size() - 1)
  {
    return {};
  }

  // get four points of the covered rectangle area
  const Eigen::Rotation2D<float> rotationMat = getRotationMat(waypoints[curIdx], waypoints[curIdx + 1]);
  const Eigen::Vector2f bottomleft = rotationMat * robotGadgetPoints[0] + waypoints[curIdx];
  const Eigen::Vector2f bottomright = rotationMat * robotGadgetPoints[1] + waypoints[curIdx];
  const Eigen::Vector2f topleft = rotationMat * robotGadgetPoints[0] + waypoints[curIdx + 1];
  const Eigen::Vector2f topright = rotationMat * robotGadgetPoints[1] + waypoints[curIdx + 1];

  // get four lines (vectors)
  const Eigen::Vector2f rightVec = topright - bottomright;
  const Eigen::Vector2f topVec = topleft - topright;
  const Eigen::Vector2f leftVec = bottomleft - topleft;
  const Eigen::Vector2f bottomtVec = bottomright - bottomleft;

  // get bounding box for the rectangle
  const float maxX = std::max(std::max(topleft[0], topright[0]), std::max(bottomleft[0], bottomright[0]));
  const float minX = std::min(std::min(topleft[0], topright[0]), std::min(bottomleft[0], bottomright[0]));
  const float maxY = std::max(std::max(topleft[1], topright[1]), std::max(bottomleft[1], bottomright[1]));
  const float minY = std::min(std::min(topleft[1], topright[1]), std::min(bottomleft[1], bottomright[1]));

  const int maxXIdx = (maxX - map.origin[0]) / map.gridSize;
  const int minXIdx = (minX - map.origin[0]) / map.gridSize;
  const int maxYIdx = (maxY - map.origin[1]) / map.gridSize;
  const int minYIdx = (minY - map.origin[1]) / map.gridSize;

  // get covered area for next iteration
  std::vector<Eigen::Vector2i> newCoveredGridIdsToNext;
  for (int x = minXIdx; x <= maxXIdx; x++)  // cols
  {
    for (int y = minYIdx; y <= maxYIdx; y++)  // rows
    {
      const Eigen::Vector2f gridCenter = map.origin + Eigen::Vector2i(x, y).cast<float>() * map.gridSize +
                                         Eigen::Vector2f(map.gridSize * 0.5f, map.gridSize * 0.5f);
      const bool isInside =
          cross2d(rightVec, (gridCenter - bottomright)) >= 0.f && cross2d(topVec, (gridCenter - topright)) >= 0.f &&
          cross2d(leftVec, (gridCenter - topleft)) >= 0.f && cross2d(bottomtVec, (gridCenter - bottomleft)) >= 0.f;
      if (isInside && !map.grids(y, x))  // rowId, colId
      {
        newCoveredGridIdsToNext.push_back(Eigen::Vector2i(x, y));
      }
    }
  }

  return newCoveredGridIdsToNext;
}

// apply 2D transformation to points
template <std::size_t N>
std::array<Eigen::Vector2f, N> estimateTransformedPoints(const std::array<Eigen::Vector2f, N>& points,
                                                         const std::vector<Eigen::Vector2f>& waypoints,
                                                         const std::size_t curIdx)
{
  std::array<Eigen::Vector2f, N> transformedPoints;
  const Eigen::Rotation2D<float> rotationMat =
      (curIdx == waypoints.size() - 1) ?
          getRotationMat(waypoints[waypoints.size() - 2], waypoints[waypoints.size() - 1]) :
          getRotationMat(waypoints[curIdx], waypoints[curIdx + 1]);
  for (std::size_t i = 0; i < N; i++)
  {
    transformedPoints[i] = rotationMat * points[i] + waypoints[curIdx];
  }
  return transformedPoints;
}

#endif  // ROBOT_PLANNER_UTILS_H