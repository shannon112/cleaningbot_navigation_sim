
#include "cleaningbot_navigation_sim/robot_vis.h"
#include <QPainter>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"

RobotVis::RobotVis(QWidget* parent) : QWidget(parent)
{
}
RobotVis::~RobotVis()
{
}

QPoint RobotVis::getPos(const Eigen::Vector2f& point)
{
  const Eigen::Vector2f pos = (point - origin_) / gridSize_;
  return QPoint(round(pos[0]), mapSizeWH_[1] - round(pos[1]));
}

void RobotVis::setupVis(const OccupancyMap& map)
{
  mapSizeWH_ = Eigen::Vector2i(map.grids.cols(), map.grids.rows());
  gridSize_ = map.gridSize;
  origin_ = map.origin;

  setFixedSize(mapSizeWH_[0], mapSizeWH_[1]);
  update();
}

void RobotVis::updateVis(const Status& status)
{
  status_ = status;
  update();  // triggers paintEvent
}

void RobotVis::paintEvent(QPaintEvent* event)
{
  if (status_ == std::nullopt)
    return;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // show position
  painter.setPen(Qt::red);
  painter.setBrush(Qt::red);
  painter.drawEllipse(getPos(status_->position), 3, 3);  // base link
  painter.setPen(Qt::blue);
  painter.setBrush(Qt::blue);
  painter.drawEllipse(getPos(status_->footprint[0]), 3, 3);
  painter.drawEllipse(getPos(status_->footprint[1]), 3, 3);
  painter.drawEllipse(getPos(status_->footprint[2]), 3, 3);
  painter.drawEllipse(getPos(status_->footprint[3]), 3, 3);
  painter.setPen(Qt::green);
  painter.setBrush(Qt::green);
  painter.drawEllipse(getPos(status_->gadget[0]), 3, 3);
  painter.drawEllipse(getPos(status_->gadget[1]), 3, 3);

  // show value
  QFont font("Arial", 12);
  painter.setFont(font);
  painter.setPen(Qt::black);

  QString velocityText = QString("%1: %2").arg("Current velocity").arg(status_->velocity);
  painter.drawText(20, 20, velocityText);
  QString distanceSoFarText = QString("%1: %2").arg("Accumulated distance").arg(status_->distanceSoFar);
  painter.drawText(20, 35, distanceSoFarText);
  QString durationSoFarText = QString("%1: %2").arg("Accumulated duration").arg(status_->durationSoFar);
  painter.drawText(20, 50, durationSoFarText);
  QString coveredAreaSoFarText = QString("%1: %2").arg("Covered area").arg(status_->coveredAreaSoFar);
  painter.drawText(20, 65, coveredAreaSoFarText);

  // ending
  QWidget::paintEvent(event);
  status_ = std::nullopt;
}
