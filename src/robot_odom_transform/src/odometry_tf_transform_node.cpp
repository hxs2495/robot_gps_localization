#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using Matrix6 = std::array<std::array<double, 6>, 6>;

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  rotation.normalize();
  return tf2::Transform(
    rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

bool valid_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(norm) && norm > 1.0e-12;
}

Matrix6 identity6()
{
  Matrix6 matrix{};
  for (std::size_t index = 0; index < 6; ++index) {
    matrix[index][index] = 1.0;
  }
  return matrix;
}

std::array<double, 36> transform_covariance(
  const std::array<double, 36> & covariance, const Matrix6 & jacobian)
{
  Matrix6 intermediate{};
  Matrix6 result{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      for (std::size_t k = 0; k < 6; ++k) {
        intermediate[row][column] +=
          jacobian[row][k] * covariance[k * 6 + column];
      }
    }
  }
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      for (std::size_t k = 0; k < 6; ++k) {
        result[row][column] += intermediate[row][k] * jacobian[column][k];
      }
    }
  }

  std::array<double, 36> output{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      output[row * 6 + column] = result[row][column];
    }
  }
  return output;
}

void set_skew_block(Matrix6 & matrix, const tf2::Vector3 & vector, double sign)
{
  const double skew[3][3] = {
    {0.0, -vector.z(), vector.y()},
    {vector.z(), 0.0, -vector.x()},
    {-vector.y(), vector.x(), 0.0},
  };
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      matrix[row][column + 3] = sign * skew[row][column];
    }
  }
}
}  // namespace

class OdometryTfTransformNode : public rclcpp::Node
{
public:
  OdometryTfTransformNode()
  : Node("odometry_tf_transform_node")
  {
    declare_parameter<std::string>("input_topic", "/Odometry");
    declare_parameter<std::string>("output_topic", "/odometry/lio/base");
    declare_parameter<std::string>("target_child_frame", "base_footprint");
    declare_parameter<std::string>("source_child_frame", "livox_imu");
    declare_parameter<double>("transform_timeout", 0.2);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    target_child_frame_ = get_parameter("target_child_frame").as_string();
    source_child_frame_ = get_parameter("source_child_frame").as_string();
    transform_timeout_ = std::max(0.0, get_parameter("transform_timeout").as_double());

    // Only static URDF transforms are queried, so they must not be invalidated
    // when a rosbag clock starts, loops, or jumps backwards.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
      std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OdometryTfTransformNode::odometry_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "里程计参考点TF转换: %s -> %s, target=%s",
      input_topic_.c_str(), output_topic_.c_str(), target_child_frame_.c_str());
  }

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!valid_pose(message->pose.pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "输入里程计位姿无效，丢弃该帧");
      return;
    }

    const std::string source_frame =
      message->child_frame_id.empty() ? source_child_frame_ : message->child_frame_id;
    if (source_frame.empty() || target_child_frame_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "里程计源或目标child frame为空");
      return;
    }

    tf2::Transform source_to_target;
    if (source_frame != target_child_frame_) {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          source_frame, target_child_frame_, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_));
        tf2::fromMsg(transform.transform, source_to_target);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "等待URDF静态TF %s <- %s: %s",
          source_frame.c_str(), target_child_frame_.c_str(), error.what());
        return;
      }
    } else {
      source_to_target.setIdentity();
    }

    const tf2::Transform parent_to_source = pose_to_transform(message->pose.pose);
    const tf2::Transform parent_to_target = parent_to_source * source_to_target;

    nav_msgs::msg::Odometry output = *message;
    output.child_frame_id = target_child_frame_;
    output.pose.pose.position.x = parent_to_target.getOrigin().x();
    output.pose.pose.position.y = parent_to_target.getOrigin().y();
    output.pose.pose.position.z = parent_to_target.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(parent_to_target.getRotation());

    // Pose covariance: moving the reference point couples orientation error
    // into position error through the rotated lever arm.
    Matrix6 pose_jacobian = identity6();
    const tf2::Vector3 lever_in_parent =
      parent_to_source.getBasis() * source_to_target.getOrigin();
    set_skew_block(pose_jacobian, lever_in_parent, -1.0);
    output.pose.covariance =
      transform_covariance(message->pose.covariance, pose_jacobian);

    // Odometry twist is expressed in child_frame_id. Shift linear velocity to
    // the target origin and rotate both linear/angular parts into target axes.
    const tf2::Vector3 linear_source(
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z);
    const tf2::Vector3 angular_source(
      message->twist.twist.angular.x,
      message->twist.twist.angular.y,
      message->twist.twist.angular.z);
    const tf2::Matrix3x3 target_from_source = source_to_target.getBasis().transpose();
    const tf2::Vector3 linear_target = target_from_source *
      (linear_source + angular_source.cross(source_to_target.getOrigin()));
    const tf2::Vector3 angular_target = target_from_source * angular_source;
    output.twist.twist.linear.x = linear_target.x();
    output.twist.twist.linear.y = linear_target.y();
    output.twist.twist.linear.z = linear_target.z();
    output.twist.twist.angular.x = angular_target.x();
    output.twist.twist.angular.y = angular_target.y();
    output.twist.twist.angular.z = angular_target.z();

    Matrix6 twist_jacobian{};
    const tf2::Vector3 lever = source_to_target.getOrigin();
    const double skew[3][3] = {
      {0.0, -lever.z(), lever.y()},
      {lever.z(), 0.0, -lever.x()},
      {-lever.y(), lever.x(), 0.0},
    };
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        const double rotation = target_from_source[row][column];
        twist_jacobian[row][column] = rotation;
        twist_jacobian[row + 3][column + 3] = rotation;
        for (std::size_t k = 0; k < 3; ++k) {
          twist_jacobian[row][column + 3] -=
            target_from_source[row][k] * skew[k][column];
        }
      }
    }
    output.twist.covariance =
      transform_covariance(message->twist.covariance, twist_jacobian);

    publisher_->publish(output);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string input_topic_;
  std::string output_topic_;
  std::string target_child_frame_;
  std::string source_child_frame_;
  double transform_timeout_{0.2};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryTfTransformNode>());
  rclcpp::shutdown();
  return 0;
}
