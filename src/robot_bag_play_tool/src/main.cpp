#include <memory>

#include <QApplication>
#include <QMetaType>

#include "rclcpp/rclcpp.hpp"

#include "robot_bag_play_tool/bag_player.hpp"
#include "robot_bag_play_tool/main_window.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  qRegisterMetaType<uint64_t>("uint64_t");
  qRegisterMetaType<int64_t>("int64_t");

  auto node = std::make_shared<rclcpp::Node>("robot_bag_play_tool");
  auto player = std::make_shared<robot_bag_play_tool::BagPlayer>(node);
  robot_bag_play_tool::MainWindow window(player);
  window.show();

  const int ret = app.exec();
  player->stopAll();
  rclcpp::shutdown();
  return ret;
}
