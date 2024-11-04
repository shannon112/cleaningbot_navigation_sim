#ifndef ROBOT_VIS_H
#define ROBOT_VIS_H

#include <QWidget>
#include <Eigen/Dense>
#include <vector>

class RobotVis : public QWidget
{
  Q_OBJECT
public:
  RobotVis(QWidget* parent = nullptr);
  ~RobotVis();

  void updateMap(const Eigen::Vector2f& origin, const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& grid,
                 const std::vector<Eigen::Vector2f>& path, const Eigen::Vector2f& position, const float mapGridSize);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  Eigen::Vector2f origin_;
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> grid_;
  std::vector<Eigen::Vector2f> path_;
  Eigen::Vector2f position_;
  float mapGridSize_;
};

#endif  // ROBOT_VIS_H