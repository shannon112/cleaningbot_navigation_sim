
#include "cleaningbot_navigation_sim/robot_vis.h"
#include <QPainter>
#include <QTimer>

RobotVis::RobotVis(QWidget* parent) : QWidget(parent)
{
  setFixedSize(800, 800);  // Set widget size, adjust as needed
}
RobotVis::~RobotVis()
{
  // Destructor implementation, even if empty
}

void RobotVis::updateMap(const Eigen::Vector2f& origin, const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& grid,
                         const std::vector<Eigen::Vector2f>& path, const Eigen::Vector2f& position,
                         const float mapGridSize)
{
  origin_ = origin;
  grid_ = grid;
  path_ = path;
  position_ = position;
  mapGridSize_ = mapGridSize;
  repaint();  // Request a paint event to redraw the widget
}

void RobotVis::paintEvent(QPaintEvent* event)
{
  (void)event;
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw the occupancy grid map
  const int cellSize = 5;  // Adjust size for each grid cell
  for (int x = 0; x < grid_.rows(); ++x)
  {
    for (int y = 0; y < grid_.cols(); ++y)
    {
      if (grid_(x, y))
      {
        painter.fillRect(x * cellSize, y * cellSize, cellSize, cellSize, Qt::black);
      }
      else
      {
        painter.fillRect(x * cellSize, y * cellSize, cellSize, cellSize, Qt::white);
      }
    }
  }

  // Draw the path
  painter.setPen(Qt::blue);
  for (const auto& point : path_)
  {
    int px = (point[0] - origin_[0]) / mapGridSize_ * cellSize;
    int py = (point[1] - origin_[1]) / mapGridSize_ * cellSize;
    painter.drawEllipse(px, py, 4, 4);
  }

  // Draw the current robot position
  painter.setPen(Qt::red);
  int posX = (position_[0] - origin_[0]) / mapGridSize_ * cellSize;
  int posY = (position_[1] - origin_[1]) / mapGridSize_ * cellSize;
  painter.drawEllipse(posX, posY, 10, 10);
}
