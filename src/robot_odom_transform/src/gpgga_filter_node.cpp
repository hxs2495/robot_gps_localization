#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class NavSatFixFilterNode : public rclcpp::Node
{
public:
  NavSatFixFilterNode()
  : Node("gpgga_filter_node")
  {
    declare_parameter<std::string>("input_topic", "/fix");
    declare_parameter<std::string>("output_topic", "/fix/filter");
    declare_parameter<int>("required_status", 0);
    declare_parameter<int>("recover_frame_count", 1);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    required_status_ = static_cast<int>(get_parameter("required_status").as_int());
    recover_frame_count_ = std::max(
      1, static_cast<int>(get_parameter("recover_frame_count").as_int()));

    subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&NavSatFixFilterNode::filter_fix, this, std::placeholders::_1));
    publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      output_topic, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(), "NavSatFix过滤节点已启动: %s -> %s, 状态>=%d, 恢复帧数=%d",
      input_topic.c_str(), output_topic.c_str(), required_status_, recover_frame_count_);
  }

private:
  void filter_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    const bool coordinates_valid =
      std::isfinite(msg->latitude) && msg->latitude >= -90.0 && msg->latitude <= 90.0 &&
      std::isfinite(msg->longitude) && msg->longitude >= -180.0 && msg->longitude <= 180.0 &&
      std::isfinite(msg->altitude);
    if (!coordinates_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "收到无效GPS坐标，丢弃该帧");
      reset_recovery();
      return;
    }

    if (msg->status.status < required_status_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "GPS定位状态不足，当前=%d，要求>=%d", msg->status.status, required_status_);
      reset_recovery();
      return;
    }

    const double x_variance = msg->position_covariance[0];
    const double y_variance = msg->position_covariance[4];
    if (!std::isfinite(x_variance) || !std::isfinite(y_variance) ||
      x_variance < 0.0 || y_variance < 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "GPS位置协方差无效，丢弃该帧");
      reset_recovery();
      return;
    }

    ++consecutive_valid_count_;
    if (consecutive_valid_count_ < recover_frame_count_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000, "等待GPS稳定: %d/%d",
        consecutive_valid_count_, recover_frame_count_);
      return;
    }

    publisher_->publish(*msg);
  }

  void reset_recovery()
  {
    consecutive_valid_count_ = 0;
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  int required_status_{0};
  int recover_frame_count_{1};
  int consecutive_valid_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavSatFixFilterNode>());
  rclcpp::shutdown();
  return 0;
}
