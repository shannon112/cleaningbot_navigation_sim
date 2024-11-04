#ifndef ROBOT_STATUS_H
#define ROBOT_STATUS_H

#include <Eigen/Dense>
#include <vector>
#include <array>

struct Status
{
  Eigen::Vector2f position = {};
  float velocity = 0.f;
  float distanceToNext = 0.f;
  float distanceSoFar = 0.f;
  float durationToNext = 0.f;
  float durationSoFar = 0.f;
  std::vector<Eigen::Vector2i> newCoveredGridIdsToNext;
  float newCoveredAreaToNext = 0.f;
  std::vector<Eigen::Vector2i> coveredGridIdsPrevToCur;
  float coveredAreaSoFar = 0.f;
  std::array<Eigen::Vector2f, 2> gadget;
  std::array<Eigen::Vector2f, 4> footprint;
};

#endif  // ROBOT_STATUS_H