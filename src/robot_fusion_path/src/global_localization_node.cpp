#include <GeographicLib/LocalCartesian.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_fusion_path/msg/global_localization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
constexpr double kQuaternionNormEpsilon = 1.0e-12;

bool valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > kQuaternionNormEpsilon;
}

bool valid_fix(const sensor_msgs::msg::NavSatFix & fix)
{
  return fix.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
         std::isfinite(fix.latitude) && std::isfinite(fix.longitude) &&
         std::isfinite(fix.altitude) && fix.latitude >= -90.0 &&
         fix.latitude <= 90.0 && fix.longitude >= -180.0 &&
         fix.longitude <= 180.0;
}

bool valid_odometry(const nav_msgs::msg::Odometry & odometry)
{
  const auto & position = odometry.pose.pose.position;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) &&
         valid_quaternion(odometry.pose.pose.orientation);
}

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation;
  tf2::fromMsg(pose.orientation, rotation);
  rotation.normalize();
  return tf2::Transform(
    rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}
}  // namespace

class GlobalLocalizationNode : public rclcpp::Node
{
public:
  GlobalLocalizationNode()
  : Node("global_localization_node")
  {
    declare_parameter<std::string>("gps_topic", "/fix/filter");
    declare_parameter<std::string>("orientation_topic", "/imu_orientation");
    declare_parameter<std::string>("odometry_topic", "/odometry/global");
    declare_parameter<std::string>("output_topic", "/localization/global");
    declare_parameter<std::string>("output_fix_topic", "/localization/global_fix");
    declare_parameter<std::string>("global_frame_id", "wgs84");
    declare_parameter<std::string>("base_frame_id", "base_footprint");
    declare_parameter<std::string>("gps_frame_id", "gps");
    declare_parameter<bool>("use_orientation", true);
    declare_parameter<std::string>("orientation_convention", "north_clockwise");
    declare_parameter<double>("orientation_yaw_offset", 0.0);
    declare_parameter<double>("initial_enu_yaw", 0.0);
    declare_parameter<bool>("two_d_mode", true);
    declare_parameter<bool>("publish_nav_sat_fix", true);
    declare_parameter<double>("max_anchor_time_offset", 0.2);
    declare_parameter<double>("transform_timeout", 0.2);
    declare_parameter<int>("anchor_buffer_size", 200);

    const auto gps_topic = get_parameter("gps_topic").as_string();
    const auto orientation_topic = get_parameter("orientation_topic").as_string();
    const auto odometry_topic = get_parameter("odometry_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    const auto output_fix_topic = get_parameter("output_fix_topic").as_string();
    global_frame_id_ = get_parameter("global_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    gps_frame_id_ = get_parameter("gps_frame_id").as_string();
    use_orientation_ = get_parameter("use_orientation").as_bool();
    orientation_convention_ = get_parameter("orientation_convention").as_string();
    orientation_yaw_offset_ = get_parameter("orientation_yaw_offset").as_double();
    initial_enu_yaw_ = get_parameter("initial_enu_yaw").as_double();
    two_d_mode_ = get_parameter("two_d_mode").as_bool();
    publish_nav_sat_fix_ = get_parameter("publish_nav_sat_fix").as_bool();
    max_anchor_time_offset_ = std::max(
      0.0, get_parameter("max_anchor_time_offset").as_double());
    transform_timeout_ = std::max(0.0, get_parameter("transform_timeout").as_double());
    anchor_buffer_size_ = std::max(
      1, static_cast<int>(get_parameter("anchor_buffer_size").as_int()));

    if (orientation_convention_ != "north_clockwise" &&
      orientation_convention_ != "enu")
    {
      throw std::invalid_argument(
              "orientation_convention必须为north_clockwise或enu");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
      std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    global_pub_ = create_publisher<robot_fusion_path::msg::GlobalLocalization>(
      output_topic, rclcpp::QoS(10));
    if (publish_nav_sat_fix_) {
      fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
        output_fix_topic, rclcpp::QoS(10));
    }

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&GlobalLocalizationNode::gps_callback, this, std::placeholders::_1));
    if (use_orientation_) {
      orientation_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        orientation_topic, rclcpp::SensorDataQoS(),
        std::bind(
          &GlobalLocalizationNode::orientation_callback, this, std::placeholders::_1));
    }
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::QoS(20),
      std::bind(&GlobalLocalizationNode::odometry_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "全球定位输出节点已启动: GPS=%s, 融合里程计=%s, 输出=%s%s",
      gps_topic.c_str(), odometry_topic.c_str(), output_topic.c_str(),
      publish_nav_sat_fix_ ? (", NavSatFix=" + output_fix_topic).c_str() : "");
    if (!use_orientation_) {
      RCLCPP_WARN(
        get_logger(),
        "未订阅航向，使用initial_enu_yaw=%.6f rad；该参数不准确会旋转整条全球轨迹。",
        initial_enu_yaw_);
    }
  }

private:
  using FixPtr = sensor_msgs::msg::NavSatFix::ConstSharedPtr;
  using ImuPtr = sensor_msgs::msg::Imu::ConstSharedPtr;

  void gps_callback(const FixPtr msg)
  {
    if (anchor_initialized_) {
      return;
    }
    if (!valid_fix(*msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "收到无效GPS Fix，不能用于全球原点初始化");
      return;
    }
    push_bounded(fix_buffer_, msg);
  }

  void orientation_callback(const ImuPtr msg)
  {
    if (anchor_initialized_) {
      return;
    }
    if (!valid_quaternion(msg->orientation)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "收到无效航向四元数，不能用于全球原点初始化");
      return;
    }
    push_bounded(orientation_buffer_, msg);
  }

  template<typename MessagePointerT>
  void push_bounded(std::deque<MessagePointerT> & buffer, const MessagePointerT & message)
  {
    buffer.push_back(message);
    while (buffer.size() > static_cast<std::size_t>(anchor_buffer_size_)) {
      buffer.pop_front();
    }
  }

  template<typename MessagePointerT>
  MessagePointerT closest_message(
    const std::deque<MessagePointerT> & buffer,
    const builtin_interfaces::msg::Time & target_stamp,
    double & time_offset) const
  {
    MessagePointerT closest;
    time_offset = std::numeric_limits<double>::infinity();
    const double target = stamp_seconds(target_stamp);
    for (const auto & message : buffer) {
      const double offset = std::abs(stamp_seconds(message->header.stamp) - target);
      if (offset < time_offset) {
        closest = message;
        time_offset = offset;
      }
    }
    return closest;
  }

  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!valid_odometry(*msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "收到无效全局融合里程计，已跳过该帧");
      return;
    }
    if (!anchor_initialized_ && !initialize_anchor(*msg)) {
      return;
    }
    publish_global_localization(*msg);
  }

  bool initialize_anchor(const nav_msgs::msg::Odometry & odometry)
  {
    double fix_offset = 0.0;
    const auto fix = closest_message(fix_buffer_, odometry.header.stamp, fix_offset);
    if (!fix || fix_offset > max_anchor_time_offset_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "等待与融合里程计同步的有效GPS（当前最近时差%s）",
        fix ? (std::to_string(fix_offset) + " s").c_str() : "未知");
      return false;
    }

    tf2::Quaternion enu_to_gps_rotation;
    builtin_interfaces::msg::Time anchor_stamp = fix->header.stamp;
    if (use_orientation_) {
      double orientation_offset = 0.0;
      const auto orientation = closest_message(
        orientation_buffer_, odometry.header.stamp, orientation_offset);
      if (!orientation || orientation_offset > max_anchor_time_offset_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "等待与融合里程计同步的有效航向（当前最近时差%s）",
          orientation ? (std::to_string(orientation_offset) + " s").c_str() : "未知");
        return false;
      }
      enu_to_gps_rotation = orientation_to_enu(orientation->orientation);
    } else {
      enu_to_gps_rotation.setRPY(0.0, 0.0, initial_enu_yaw_);
    }

    tf2::Transform gps_to_base;
    if (gps_frame_id_ == base_frame_id_) {
      gps_to_base.setIdentity();
    } else {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          gps_frame_id_, base_frame_id_, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_));
        tf2::fromMsg(transform.transform, gps_to_base);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "等待URDF静态TF %s <- %s: %s",
          gps_frame_id_.c_str(), base_frame_id_.c_str(), error.what());
        return false;
      }
    }

    local_cartesian_.Reset(fix->latitude, fix->longitude, fix->altitude);
    const tf2::Transform enu_to_gps(enu_to_gps_rotation, tf2::Vector3(0.0, 0.0, 0.0));
    const tf2::Transform enu_to_base = enu_to_gps * gps_to_base;
    const tf2::Transform map_to_base = pose_to_transform(odometry.pose.pose);
    enu_to_map_ = enu_to_base * map_to_base.inverse();

    anchor_fix_status_ = fix->status.status;
    anchor_fix_service_ = fix->status.service;
    anchor_stamp_ = anchor_stamp;
    anchor_latitude_ = fix->latitude;
    anchor_longitude_ = fix->longitude;
    anchor_altitude_ = fix->altitude;
    anchor_initialized_ = true;
    fix_buffer_.clear();
    orientation_buffer_.clear();

    double base_latitude = 0.0;
    double base_longitude = 0.0;
    double base_altitude = 0.0;
    local_cartesian_.Reverse(
      enu_to_base.getOrigin().x(), enu_to_base.getOrigin().y(),
      enu_to_base.getOrigin().z(), base_latitude, base_longitude, base_altitude);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(enu_to_base.getRotation()).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
      get_logger(),
      "全球原点初始化完成: antenna=[%.9f, %.9f, %.3f], "
      "base=[%.9f, %.9f, %.3f], ENU yaw=%.6f rad, GPS/odom时差=%.3f s",
      anchor_latitude_, anchor_longitude_, anchor_altitude_, base_latitude,
      base_longitude, base_altitude, yaw, fix_offset);
    return true;
  }

  tf2::Quaternion orientation_to_enu(
    const geometry_msgs::msg::Quaternion & message_orientation) const
  {
    tf2::Quaternion source;
    tf2::fromMsg(message_orientation, source);
    source.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(source).getRPY(roll, pitch, yaw);
    const double enu_yaw = orientation_convention_ == "enu" ? yaw : M_PI_2 - yaw;

    tf2::Quaternion converted;
    converted.setRPY(
      two_d_mode_ ? 0.0 : roll, two_d_mode_ ? 0.0 : pitch,
      enu_yaw + orientation_yaw_offset_);
    converted.normalize();
    return converted;
  }

  void publish_global_localization(const nav_msgs::msg::Odometry & odometry)
  {
    const tf2::Transform map_to_base = pose_to_transform(odometry.pose.pose);
    tf2::Transform enu_to_base = enu_to_map_ * map_to_base;
    if (two_d_mode_) {
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(enu_to_base.getRotation()).getRPY(roll, pitch, yaw);
      tf2::Quaternion planar_rotation;
      planar_rotation.setRPY(0.0, 0.0, yaw);
      enu_to_base.setRotation(planar_rotation);
    }

    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    local_cartesian_.Reverse(
      enu_to_base.getOrigin().x(), enu_to_base.getOrigin().y(),
      enu_to_base.getOrigin().z(), latitude, longitude, altitude);

    robot_fusion_path::msg::GlobalLocalization output;
    output.header = odometry.header;
    output.header.frame_id = global_frame_id_;
    output.child_frame_id =
      odometry.child_frame_id.empty() ? base_frame_id_ : odometry.child_frame_id;
    output.latitude = latitude;
    output.longitude = longitude;
    output.altitude = altitude;
    output.orientation = tf2::toMsg(enu_to_base.getRotation());
    output.twist = odometry.twist.twist;
    rotate_covariance(odometry.pose.covariance, output.pose_covariance);
    output.twist_covariance = odometry.twist.covariance;
    output.anchor_fix_status = anchor_fix_status_;
    output.anchor_fix_service = anchor_fix_service_;
    output.satellites_used = robot_fusion_path::msg::GlobalLocalization::SATELLITES_UNKNOWN;
    output.anchor_stamp = anchor_stamp_;
    global_pub_->publish(output);

    if (fix_pub_) {
      sensor_msgs::msg::NavSatFix fix;
      fix.header = output.header;
      // This is the geodetic position of the fused body reference point, not
      // a new measurement located at the GNSS antenna.
      fix.header.frame_id = output.child_frame_id;
      fix.status.status = anchor_fix_status_;
      fix.status.service = anchor_fix_service_;
      fix.latitude = latitude;
      fix.longitude = longitude;
      fix.altitude = altitude;
      fix.position_covariance[0] = output.pose_covariance[0];
      fix.position_covariance[1] = output.pose_covariance[1];
      fix.position_covariance[2] = output.pose_covariance[2];
      fix.position_covariance[3] = output.pose_covariance[6];
      fix.position_covariance[4] = output.pose_covariance[7];
      fix.position_covariance[5] = output.pose_covariance[8];
      fix.position_covariance[6] = output.pose_covariance[12];
      fix.position_covariance[7] = output.pose_covariance[13];
      fix.position_covariance[8] = output.pose_covariance[14];
      fix.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
      fix_pub_->publish(fix);
    }
  }

  void rotate_covariance(
    const std::array<double, 36> & input,
    std::array<double, 36> & output) const
  {
    tf2::Matrix3x3 rotation(enu_to_map_.getRotation());
    double jacobian[6][6] = {};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        jacobian[row][column] = rotation[row][column];
        jacobian[row + 3][column + 3] = rotation[row][column];
      }
    }

    double intermediate[6][6] = {};
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        for (int k = 0; k < 6; ++k) {
          intermediate[row][column] += jacobian[row][k] * input[k * 6 + column];
        }
      }
    }
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        output[row * 6 + column] = 0.0;
        for (int k = 0; k < 6; ++k) {
          output[row * 6 + column] += intermediate[row][k] * jacobian[column][k];
        }
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<robot_fusion_path::msg::GlobalLocalization>::SharedPtr global_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::deque<FixPtr> fix_buffer_;
  std::deque<ImuPtr> orientation_buffer_;
  GeographicLib::LocalCartesian local_cartesian_;
  tf2::Transform enu_to_map_;
  builtin_interfaces::msg::Time anchor_stamp_;

  std::string global_frame_id_;
  std::string base_frame_id_;
  std::string gps_frame_id_;
  std::string orientation_convention_;
  bool use_orientation_{true};
  bool two_d_mode_{true};
  bool publish_nav_sat_fix_{true};
  bool anchor_initialized_{false};
  double orientation_yaw_offset_{0.0};
  double initial_enu_yaw_{0.0};
  double max_anchor_time_offset_{0.2};
  double transform_timeout_{0.2};
  double anchor_latitude_{0.0};
  double anchor_longitude_{0.0};
  double anchor_altitude_{0.0};
  int anchor_buffer_size_{200};
  int8_t anchor_fix_status_{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
  uint16_t anchor_fix_service_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
