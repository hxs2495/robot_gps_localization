#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
bool is_valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-12;
}

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  rotation.normalize();
  return tf2::Transform(
    rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}
}  // namespace

class GpsGlobalOdometryNode : public rclcpp::Node
{
public:
  GpsGlobalOdometryNode()
  : Node("gps_global_odometry_node")
  {
    declare_parameter<std::string>("input_odom_topic", "/utm/gps");
    declare_parameter<std::string>("output_odom_topic", "/odometry/gps");
    declare_parameter<std::string>("path_topic", "/gps_path");
    declare_parameter<std::string>("global_frame_id", "map");
    declare_parameter<std::string>("child_frame_id", "base_footprint");
    declare_parameter<int>("init_frames", 10);
    declare_parameter<bool>("two_d_mode", true);
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<bool>("publish_path", true);
    declare_parameter<double>("path_min_distance", 0.05);
    declare_parameter<int>("path_max_points", 10000);

    // T_gps_base: GPS天线坐标系到机器人基座坐标系的标定外参。
    declare_parameter<double>("gps_to_base.x", 0.726732);
    declare_parameter<double>("gps_to_base.y", -0.109451);
    declare_parameter<double>("gps_to_base.z", 0.0);
    declare_parameter<double>("gps_to_base.qx", 0.00259991);
    declare_parameter<double>("gps_to_base.qy", 0.00764529);
    declare_parameter<double>("gps_to_base.qz", -0.257884);
    declare_parameter<double>("gps_to_base.qw", 0.966142);

    input_odom_topic_ = get_parameter("input_odom_topic").as_string();
    output_odom_topic_ = get_parameter("output_odom_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    global_frame_id_ = get_parameter("global_frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();
    init_frames_ = std::max(1, static_cast<int>(get_parameter("init_frames").as_int()));
    two_d_mode_ = get_parameter("two_d_mode").as_bool();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    publish_path_ = get_parameter("publish_path").as_bool();
    path_min_distance_ = std::max(0.0, get_parameter("path_min_distance").as_double());
    path_max_points_ =
      std::max(0, static_cast<int>(get_parameter("path_max_points").as_int()));

    tf2::Quaternion gps_to_base_rotation(
      get_parameter("gps_to_base.qx").as_double(),
      get_parameter("gps_to_base.qy").as_double(),
      get_parameter("gps_to_base.qz").as_double(),
      get_parameter("gps_to_base.qw").as_double());
    if (gps_to_base_rotation.length2() < 1.0e-12) {
      RCLCPP_WARN(
        get_logger(), "gps_to_base四元数无效，已回退为单位旋转");
      gps_to_base_rotation.setRPY(0.0, 0.0, 0.0);
    } else {
      gps_to_base_rotation.normalize();
    }
    gps_to_base_ = tf2::Transform(
      gps_to_base_rotation,
      tf2::Vector3(
        get_parameter("gps_to_base.x").as_double(),
        get_parameter("gps_to_base.y").as_double(),
        get_parameter("gps_to_base.z").as_double()));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    if (publish_path_) {
      const auto path_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GpsGlobalOdometryNode::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "GPS全局里程计节点已启动: %s -> %s, frame=%s, child=%s, 初始化帧数=%d",
      input_odom_topic_.c_str(), output_odom_topic_.c_str(),
      global_frame_id_.c_str(), child_frame_id_.c_str(), init_frames_);
    if (path_pub_) {
      RCLCPP_INFO(
        get_logger(),
        "GPS路径发布: %s, 最小采样距离=%.3f m, 最大点数=%d%s",
        path_topic_.c_str(), path_min_distance_, path_max_points_,
        path_max_points_ == 0 ? "（不限制）" : "");
    }
    RCLCPP_INFO(
      get_logger(),
      "使用T_gps_base标定: xyz=[%.6f, %.6f, %.6f], xyzw=[%.6f, %.6f, %.6f, %.6f]",
      gps_to_base_.getOrigin().x(), gps_to_base_.getOrigin().y(),
      gps_to_base_.getOrigin().z(), gps_to_base_.getRotation().x(),
      gps_to_base_.getRotation().y(), gps_to_base_.getRotation().z(),
      gps_to_base_.getRotation().w());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & position = msg->pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "收到的UTM里程计位置无效，丢弃该帧");
      return;
    }
    if (!is_valid_quaternion(msg->pose.pose.orientation)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "收到的UTM里程计四元数无效，丢弃该帧");
      return;
    }

    // 输入是T_utm_gps，应用标定后得到GPS观测下的机器人基座位姿T_utm_base。
    const tf2::Transform utm_to_gps = pose_to_transform(msg->pose.pose);
    const tf2::Transform utm_to_base = utm_to_gps * gps_to_base_;

    if (!origin_initialized_) {
      collect_initial_pose(utm_to_base);
      if (collected_count_ < init_frames_) {
        RCLCPP_INFO(
          get_logger(), "采集GPS全局坐标原点 %d/%d", collected_count_, init_frames_);
        return;
      }
      initialize_origin();
    }

    publish_global_odometry(*msg, utm_to_base);
  }

  void collect_initial_pose(const tf2::Transform & utm_to_base)
  {
    const auto & position = utm_to_base.getOrigin();
    sum_x_ += position.x();
    sum_y_ += position.y();
    sum_z_ += position.z();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(utm_to_base.getRotation()).getRPY(roll, pitch, yaw);
    sum_sin_yaw_ += std::sin(yaw);
    sum_cos_yaw_ += std::cos(yaw);
    ++collected_count_;
  }

  void initialize_origin()
  {
    const double count = static_cast<double>(collected_count_);
    const double initial_yaw = std::atan2(sum_sin_yaw_ / count, sum_cos_yaw_ / count);
    tf2::Quaternion initial_rotation;
    initial_rotation.setRPY(0.0, 0.0, initial_yaw);

    // T_global_utm使机器人启动位姿在global_frame_id中成为原点和单位朝向。
    const tf2::Transform utm_to_global_origin(
      initial_rotation,
      tf2::Vector3(sum_x_ / count, sum_y_ / count, sum_z_ / count));
    global_to_utm_ = utm_to_global_origin.inverse();
    origin_initialized_ = true;

    RCLCPP_INFO(
      get_logger(),
      "GPS全局坐标原点初始化完成: UTM=[%.3f, %.3f, %.3f], yaw=%.3f rad",
      sum_x_ / count, sum_y_ / count, sum_z_ / count, initial_yaw);
  }

  void publish_global_odometry(
    const nav_msgs::msg::Odometry & input, const tf2::Transform & utm_to_base)
  {
    tf2::Transform global_to_base = global_to_utm_ * utm_to_base;
    if (two_d_mode_) {
      auto position = global_to_base.getOrigin();
      position.setZ(0.0);
      global_to_base.setOrigin(position);

      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(global_to_base.getRotation()).getRPY(roll, pitch, yaw);
      tf2::Quaternion planar_rotation;
      planar_rotation.setRPY(0.0, 0.0, yaw);
      global_to_base.setRotation(planar_rotation);
    }

    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = global_frame_id_;
    output.child_frame_id = child_frame_id_;
    output.pose.pose.position.x = global_to_base.getOrigin().x();
    output.pose.pose.position.y = global_to_base.getOrigin().y();
    output.pose.pose.position.z = global_to_base.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(global_to_base.getRotation());
    rotate_pose_covariance(input.pose.covariance, output.pose.covariance);
    odom_pub_->publish(output);
    publish_path(output);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = output.child_frame_id;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_path(const nav_msgs::msg::Odometry & odometry)
  {
    if (!path_pub_) {
      return;
    }

    const auto & position = odometry.pose.pose.position;
    if (!gps_path_.poses.empty() && path_min_distance_ > 0.0) {
      const auto & previous = gps_path_.poses.back().pose.position;
      const double dx = position.x - previous.x;
      const double dy = position.y - previous.y;
      const double dz = position.z - previous.z;
      if (std::hypot(std::hypot(dx, dy), dz) < path_min_distance_) {
        return;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odometry.header;
    pose.pose = odometry.pose.pose;
    gps_path_.header = odometry.header;
    gps_path_.poses.push_back(pose);

    if (path_max_points_ > 0 &&
      gps_path_.poses.size() > static_cast<std::size_t>(path_max_points_))
    {
      const auto remove_count =
        gps_path_.poses.size() - static_cast<std::size_t>(path_max_points_);
      gps_path_.poses.erase(
        gps_path_.poses.begin(), gps_path_.poses.begin() + remove_count);
    }
    path_pub_->publish(gps_path_);
  }

  void rotate_pose_covariance(
    const std::array<double, 36> & input, std::array<double, 36> & output) const
  {
    tf2::Matrix3x3 rotation(global_to_utm_.getRotation());
    double rotation6[6][6] = {};
    for (int row = 0; row < 6; ++row) {
      rotation6[row][row] = 1.0;
    }
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        rotation6[row][column] = rotation[row][column];
        rotation6[row + 3][column + 3] = rotation[row][column];
      }
    }

    double intermediate[6][6] = {};
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        for (int k = 0; k < 6; ++k) {
          intermediate[row][column] += rotation6[row][k] * input[k * 6 + column];
        }
      }
    }
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        output[row * 6 + column] = 0.0;
        for (int k = 0; k < 6; ++k) {
          output[row * 6 + column] += intermediate[row][k] * rotation6[column][k];
        }
      }
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string path_topic_;
  std::string global_frame_id_;
  std::string child_frame_id_;
  int init_frames_{10};
  bool two_d_mode_{true};
  bool publish_tf_{false};
  bool publish_path_{true};
  double path_min_distance_{0.05};
  int path_max_points_{10000};

  tf2::Transform gps_to_base_;
  tf2::Transform global_to_utm_;
  nav_msgs::msg::Path gps_path_;
  int collected_count_{0};
  double sum_x_{0.0};
  double sum_y_{0.0};
  double sum_z_{0.0};
  double sum_sin_yaw_{0.0};
  double sum_cos_yaw_{0.0};
  bool origin_initialized_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsGlobalOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
