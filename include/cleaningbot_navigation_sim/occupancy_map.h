#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#include <Eigen/Dense>

struct OccupancyMap
{
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> grids;
  Eigen::Vector2f origin = { 0.f, 0.f };
  float gridSize = 0.f;
};

#endif