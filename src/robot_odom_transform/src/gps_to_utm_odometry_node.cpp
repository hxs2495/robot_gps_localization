#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <GeographicLib/UTMUPS.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GpsToUtmOdometryNode : public rclcpp::Node
{
public:
  GpsToUtmOdometryNode()
  : Node("gps_to_utm_odometry_node")
  {
    declare_parameter<std::string>("gps_topic", "/fix/filter");
    declare_parameter<std::string>("orient_topic", "/imu_orientation");
    declare_parameter<std::string>("odom_topic", "/utm/gps");
    declare_parameter<std::string>("frame_id", "utm");
    declare_parameter<std::string>("child_frame_id", "gps");
    declare_parameter<bool>("use_orientation", true);
    declare_parameter<double>("orientation_variance", 0.04);
    declare_parameter<double>("unobserved_orientation_variance", 1.0e6);
    declare_parameter<double>("position_variance_floor", 0.01);
    declare_parameter<double>("unknown_position_variance", 4.0);
    declare_parameter<int>("sync_queue_size", 20);
    declare_parameter<double>("max_orientation_time_offset", 0.05);

    const auto gps_topic = get_parameter("gps_topic").as_string();
    const auto orientation_topic = get_parameter("orient_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();
    use_orientation_ = get_parameter("use_orientation").as_bool();
    orientation_variance_ = std::max(
      0.0, get_parameter("orientation_variance").as_double());
    unobserved_orientation_variance_ = std::max(
      0.0, get_parameter("unobserved_orientation_variance").as_double());
    position_variance_floor_ = std::max(
      1.0e-9, get_parameter("position_variance_floor").as_double());
    unknown_position_variance_ = std::max(
      position_variance_floor_, get_parameter("unknown_position_variance").as_double());
    sync_queue_size_ = std::max(2, static_cast<int>(get_parameter("sync_queue_size").as_int()));
    max_orientation_time_offset_ = std::max(
      0.001, get_parameter("max_orientation_time_offset").as_double());
    last_orientation_variance_ = orientation_variance_;

    if (use_orientation_) {
      gps_filter_sub_.subscribe(this, gps_topic, rmw_qos_profile_sensor_data);
      orientation_filter_sub_.subscribe(
        this, orientation_topic, rmw_qos_profile_sensor_data);
      SyncPolicy policy(sync_queue_size_);
      policy.setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(max_orientation_time_offset_));
      synchronizer_ = std::make_shared<Synchronizer>(
        static_cast<const SyncPolicy &>(policy),
        gps_filter_sub_, orientation_filter_sub_);
      synchronizer_->registerCallback(
        std::bind(
          &GpsToUtmOdometryNode::convert_synchronized,
          this, std::placeholders::_1, std::placeholders::_2));
    } else {
      gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic, rclcpp::SensorDataQoS(),
        std::bind(&GpsToUtmOdometryNode::convert_fix, this, std::placeholders::_1));
    }
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    RCLCPP_INFO(
      get_logger(), "GPS转UTM节点已启动: %s -> %s, 航向=%s",
      gps_topic.c_str(), odom_topic.c_str(), use_orientation_ ? orientation_topic.c_str() : "关闭");
  }

private:
  void convert_synchronized(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr & fix,
    const sensor_msgs::msg::Imu::ConstSharedPtr & orientation)
  {
    if (update_orientation(orientation)) {
      convert_fix(fix);
    }
  }

  bool update_orientation(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
  {
    tf2::Quaternion orientation(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    if (!std::isfinite(orientation.length2()) || orientation.length2() < 1.0e-12) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "收到无效航向四元数，忽略该帧");
      return false;
    }
    orientation.normalize();

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    tf2::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);

    // 输入航向沿用原链路约定：转换为ENU中从东向北逆时针的yaw。
    yaw_ = M_PI_2 - yaw;
    if (yaw_ > M_PI) {
      yaw_ -= 2.0 * M_PI;
    }

    const double message_variance = msg->orientation_covariance[8];
    last_orientation_variance_ =
      std::isfinite(message_variance) && message_variance > 0.0 ?
      message_variance : orientation_variance_;
    has_orientation_ = true;
    return true;
  }

  void convert_fix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & msg)
  {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "GPS无有效定位，跳过该帧");
      return;
    }
    try {
      double easting = 0.0;
      double northing = 0.0;
      int zone = 0;
      bool north = false;
      GeographicLib::UTMUPS::Forward(
        msg->latitude, msg->longitude, zone, north, easting, northing);

      nav_msgs::msg::Odometry output;
      output.header = msg->header;
      output.header.frame_id = frame_id_;
      output.child_frame_id = child_frame_id_;
      output.pose.pose.position.x = easting;
      output.pose.pose.position.y = northing;
      output.pose.pose.position.z = msg->altitude;

      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, use_orientation_ ? yaw_ : 0.0);
      output.pose.pose.orientation.x = orientation.x();
      output.pose.pose.orientation.y = orientation.y();
      output.pose.pose.orientation.z = orientation.z();
      output.pose.pose.orientation.w = orientation.w();

      fill_pose_covariance(*msg, output);
      odometry_pub_->publish(output);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "UTM转换失败: %s", error.what());
    }
  }

  void fill_pose_covariance(
    const sensor_msgs::msg::NavSatFix & fix, nav_msgs::msg::Odometry & odometry)
  {
    odometry.pose.covariance.fill(0.0);

    const bool covariance_known =
      fix.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    const bool horizontal_covariance_valid =
      std::isfinite(fix.position_covariance[0]) && fix.position_covariance[0] > 0.0 &&
      std::isfinite(fix.position_covariance[4]) && fix.position_covariance[4] > 0.0;

    if (covariance_known && horizontal_covariance_valid) {
      for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
          const double value = fix.position_covariance[row * 3 + column];
          odometry.pose.covariance[row * 6 + column] =
            std::isfinite(value) ? value : 0.0;
        }
      }
      odometry.pose.covariance[0] =
        std::max(position_variance_floor_, odometry.pose.covariance[0]);
      odometry.pose.covariance[7] =
        std::max(position_variance_floor_, odometry.pose.covariance[7]);
      odometry.pose.covariance[14] =
        std::max(position_variance_floor_, odometry.pose.covariance[14]);
    } else {
      odometry.pose.covariance[0] = unknown_position_variance_;
      odometry.pose.covariance[7] = unknown_position_variance_;
      odometry.pose.covariance[14] = unknown_position_variance_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "GPS位置协方差未知或无效，使用回退方差 %.3f m^2", unknown_position_variance_);
    }

    odometry.pose.covariance[21] = unobserved_orientation_variance_;
    odometry.pose.covariance[28] = unobserved_orientation_variance_;
    odometry.pose.covariance[35] =
      use_orientation_ && has_orientation_ ?
      last_orientation_variance_ : unobserved_orientation_variance_;
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::NavSatFix, sensor_msgs::msg::Imu>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_filter_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Imu> orientation_filter_sub_;
  std::shared_ptr<Synchronizer> synchronizer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  std::string frame_id_;
  std::string child_frame_id_;
  bool use_orientation_{true};
  bool has_orientation_{false};
  double yaw_{0.0};
  double orientation_variance_{0.04};
  double last_orientation_variance_{0.04};
  double unobserved_orientation_variance_{1.0e6};
  double position_variance_floor_{0.01};
  double unknown_position_variance_{4.0};
  int sync_queue_size_{20};
  double max_orientation_time_offset_{0.05};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsToUtmOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
