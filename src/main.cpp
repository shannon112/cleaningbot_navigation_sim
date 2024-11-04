#include "cleaningbot_navigation_sim/robot_planner.h"
#include "cleaningbot_navigation_sim/robot_vis.h"

#include <QApplication>
#include <thread>
#include <csignal>
#include <QTimer>

std::atomic<bool> shutdown_requested(false);
void signalHandler(int)
{
  shutdown_requested.store(true);
}

int main(int argc, char* argv[])
{
  std::signal(SIGINT, signalHandler);
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  // visualizer
  auto visWidget = std::make_shared<RobotVis>();
  visWidget->show();

  // planner node
  auto planner = std::make_shared<RobotPlanner>(visWidget);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(planner);
  std::thread spinThread([&executor]() { executor.spin(); });

  // handler ctrl+c
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]() {
    if (shutdown_requested.load())
    {
      app.quit();
    }
  });
  timer.start(100);

  // ending
  int result = app.exec();
  spinThread.join();
  rclcpp::shutdown();
  return result;
}
