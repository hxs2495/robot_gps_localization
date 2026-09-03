#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class FusionPathNode : public rclcpp::Node
{
public:
  FusionPathNode()
  : Node("fusion_path_node")
  {
    declare_parameter<std::string>("odom_topic", "/odometry/global");
    declare_parameter<std::string>("path_topic", "/global_path");
    declare_parameter<std::string>("path_frame_id", "");
    declare_parameter<std::string>("local_odom_topic", "/odometry/local");
    declare_parameter<std::string>("local_path_topic", "/local_path");
    declare_parameter<std::string>("local_path_frame_id", "");
    declare_parameter<int>("max_path_size", 0);
    declare_parameter<double>("min_distance", 0.0);

    const auto odom_topic = get_parameter("odom_topic").as_string();
    const auto path_topic = get_parameter("path_topic").as_string();
    const auto local_odom_topic = get_parameter("local_odom_topic").as_string();
    const auto local_path_topic = get_parameter("local_path_topic").as_string();
    path_frame_id_ = get_parameter("path_frame_id").as_string();
    local_path_frame_id_ = get_parameter("local_path_frame_id").as_string();
    max_path_size_ = get_parameter("max_path_size").as_int();
    min_distance_ = std::max(0.0, get_parameter("min_distance").as_double());

    if (max_path_size_ < 0) {
      RCLCPP_WARN(get_logger(), "max_path_size不能小于0，已按0处理为不限制路径长度。");
      max_path_size_ = 0;
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(10));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      rclcpp::QoS(10),
      std::bind(&FusionPathNode::globalOdomCallback, this, std::placeholders::_1));

    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic, rclcpp::QoS(10));
    local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic,
      rclcpp::QoS(10),
      std::bind(&FusionPathNode::localOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "融合路径节点已启动，全局订阅: %s，发布: %s；局部订阅: %s，发布: %s",
      odom_topic.c_str(),
      path_topic.c_str(),
      local_odom_topic.c_str(),
      local_path_topic.c_str());
  }

private:
  void globalOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    updatePath(msg, path_frame_id_, "全局", path_, path_pub_);
  }

  void localOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    updatePath(msg, local_path_frame_id_, "局部", local_path_, local_path_pub_);
  }

  void updatePath(
    const nav_msgs::msg::Odometry::SharedPtr msg,
    const std::string & configured_frame_id,
    const char * path_name,
    nav_msgs::msg::Path & path,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher)
  {
    if (!isFinitePose(msg->pose.pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "收到无效%s里程计位姿，已跳过该帧。", path_name);
      return;
    }

    const std::string frame_id =
      configured_frame_id.empty() ? msg->header.frame_id : configured_frame_id;

    if (frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "%s里程计header.frame_id为空，发布的Path也将没有坐标系。", path_name);
    }

    if (!path.header.frame_id.empty() && path.header.frame_id != frame_id) {
      RCLCPP_WARN(
        get_logger(),
        "%sPath坐标系从%s变为%s，已清空旧路径。",
        path_name,
        path.header.frame_id.c_str(),
        frame_id.c_str());
      path.poses.clear();
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = frame_id;
    pose.pose = msg->pose.pose;

    if (shouldAppendPose(pose, path)) {
      path.poses.push_back(pose);
      trimPath(path);
    }

    path.header.stamp = msg->header.stamp;
    path.header.frame_id = frame_id;
    publisher->publish(path);
  }

  bool isFinitePose(const geometry_msgs::msg::Pose & pose) const
  {
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
  }

  bool shouldAppendPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path) const
  {
    if (path.poses.empty() || min_distance_ <= 0.0) {
      return true;
    }

    const auto & last = path.poses.back().pose.position;
    const auto & current = pose.pose.position;
    const double dx = current.x - last.x;
    const double dy = current.y - last.y;
    const double dz = current.z - last.z;
    return dx * dx + dy * dy + dz * dz >= min_distance_ * min_distance_;
  }

  void trimPath(nav_msgs::msg::Path & path)
  {
    if (max_path_size_ <= 0) {
      return;
    }

    while (static_cast<int>(path.poses.size()) > max_path_size_) {
      path.poses.erase(path.poses.begin());
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path local_path_;

  std::string path_frame_id_;
  std::string local_path_frame_id_;
  int max_path_size_{0};
  double min_distance_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionPathNode>());
  rclcpp::shutdown();
  return 0;
}
