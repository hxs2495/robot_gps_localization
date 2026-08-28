#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath> 
constexpr double RAD2DEG = 180.0 / M_PI;

class OdomToTFNode : public rclcpp::Node
{
public:
    OdomToTFNode()
        : Node("odom_to_tf_node")
    {
        // 参数声明
        this->declare_parameter<std::string>("odom_topic", "/odometry/gps");
        this->declare_parameter<std::string>("imu_topic", "/imu_orientation");
        this->declare_parameter<std::string>("odom_frame", "camera_init");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("tf_pub_topic", "/odometry_tf");

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        tf_pub_topic_ = this->get_parameter("tf_pub_topic").as_string();

        // 订阅里程计
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&OdomToTFNode::odomCallback, this, std::placeholders::_1));

        // 订阅IMU
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&OdomToTFNode::imuCallback, this, std::placeholders::_1));

        // tf广播器
        // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // 额外发布TF消息的话题
        tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(tf_pub_topic_, 10);
        // 打印中文日志
        RCLCPP_INFO(this->get_logger(),
                    "里程转换TF节点已经启动，订阅: %s, %s, 发布: %s",
                    odom_topic.c_str(), imu_topic.c_str(), tf_pub_topic_.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_quat_ = msg->orientation;
        // 四元素转换为欧拉角
        tf2::Quaternion q(
            latest_imu_quat_.x, latest_imu_quat_.y,
            latest_imu_quat_.z, latest_imu_quat_.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        yaw = 90 - yaw * RAD2DEG; // 转换为角度
        if (yaw > 180) {
            yaw -= 360; // 确保角度在 -180 到 180 度之间
        }
        // 欧拉角转四元数，并更新最新的IMU四元数
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(roll, pitch, yaw / RAD2DEG); // 将角度转回弧度并设置RPY
        latest_imu_quat_ = tf2::toMsg(tf2_quat); // 转换为geometry_msgs::msg::Quaternion
        

        // latest_imu_quat_.x = 0;
        // latest_imu_quat_.y = sin(yaw / 2);
        // latest_imu_quat_.z = 0; // 假设只需要yaw
        // latest_imu_quat_.w = cos(yaw / 2);
        // 标记有IMU数据
        RCLCPP_INFO(this->get_logger(), "接收imu消息");
        has_imu_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = msg->header.stamp; // 保留原始消息的时间戳
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;

        // translation: 使用最新的里程计位置
        tf_msg.transform.translation.x = msg->pose.pose.position.x;
        tf_msg.transform.translation.y = msg->pose.pose.position.y;
        tf_msg.transform.translation.z = msg->pose.pose.position.z;

        // rotation: 使用最新imu四元数
        {
            // std::lock_guard<std::mutex> lock(imu_mutex_);
            // odometry/gps 已经转到x朝东
            // if (has_imu_)
            // {
            //     tf_msg.transform.rotation = latest_imu_quat_;
            // }
            // else
            // {
                tf_msg.transform.rotation = msg->pose.pose.orientation; // fallback
            // }
        }
        // tf_msg.transform.rotation = latest_imu_quat_;
        //  通过tf broadcaster发布
        //  tf_broadcaster_->sendTransform(tf_msg);

        // 额外发布TransformStamped消息到tf_pub_topic_
        tf_pub_->publish(tf_msg);
        RCLCPP_INFO(this->get_logger(), "发布TF: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_pub_;
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string odom_frame_;
    std::string base_frame_;
    std::string tf_pub_topic_;

    geometry_msgs::msg::Quaternion latest_imu_quat_;
    bool has_imu_ = false;
    std::mutex imu_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
