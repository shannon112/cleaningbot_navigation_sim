#ifndef ROBOT_VIS_H
#define ROBOT_VIS_H

#include "cleaningbot_navigation_sim/robot_status.h"
#include "cleaningbot_navigation_sim/occupancy_map.h"

#include <QWidget>
#include <Eigen/Dense>

#include <optional>

class RobotVis : public QWidget
{
  Q_OBJECT
public:
  RobotVis(QWidget* parent = nullptr);
  ~RobotVis();
  void setupVis(const OccupancyMap& map, const std::vector<Eigen::Vector2f>& waypoints);
  void updateVis(const Status& status);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  QPoint getPos(const Eigen::Vector2f& point);
  QPoint getPos(const Eigen::Vector2i& point);

  QPixmap pixelMapGrids_;
  QPixmap pixelMapTrajectory_;

  std::optional<Status> status_;
  float gridSize_;
  Eigen::Vector2i mapSizeWH_;
  Eigen::Vector2f origin_;
};

#endif  // ROBOT_VIS_H