#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

class OdometryTransformer : public rclcpp::Node
{
public:
    OdometryTransformer() : Node("odometry_transformer")
    {

        // 参数声明
        this->declare_parameter<std::string>("input_topic", "/utm/gps_new");
        this->declare_parameter<std::string>("output_topic", "/odom_in_map");

        get_parameter("input_topic", input_odom_topic_);
        get_parameter("output_topic", output_topic);
        // 初始化tf缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建Odometry订阅者
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_, 10, std::bind(&OdometryTransformer::odom_callback, this, std::placeholders::_1));

        // 创建Odometry发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
            
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        try
        {
            // 获取从UTM到camera_init的tf转换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("camera_init", msg->header.frame_id, tf2::TimePointZero);
            geometry_msgs::msg::TransformStamped tf_body2gps = tf_buffer_->lookupTransform("gps", "gps_body", tf2::TimePointZero);
            geometry_msgs::msg::TransformStamped transform2 = tf_buffer_->lookupTransform("gps_init", msg->header.frame_id, tf2::TimePointZero);
            std::cout << "查找成功" << std::endl;
            // 创建一个PoseStamped消息，用于转换
            geometry_msgs::msg::PoseStamped pose_in_odom;
            pose_in_odom.header = msg->header;
            pose_in_odom.pose = msg->pose.pose;

            // 转换Pose到map坐标系
            geometry_msgs::msg::PoseStamped pose_in_camera_init;
            tf2::doTransform(pose_in_odom, pose_in_camera_init, transform);

            // 创建一个新的Odometry消息，其frame_id为camera_init
            nav_msgs::msg::Odometry odom_in_camera_init = *msg;
            odom_in_camera_init.header.frame_id = "camera_init";
            // odom_in_camera_init.child_frame_id = "gps";
            odom_in_camera_init.pose.pose = pose_in_camera_init.pose;
			odom_in_camera_init.pose.pose.position.z = 0.0;
            
            
            // lijiabin 25/8/12  此前朝向为天线朝向在camera_init坐标系下的观测，其本身依然是天线的朝向
            //                   订阅body到gps旋转来转换成车体朝向
            double delta_yaw = tf2::getYaw(tf_body2gps.transform.rotation);
            // RCLCPP_INFO(this->get_logger(), "body到GPS的偏航角: %.2f", delta_yaw);
            tf2::Quaternion q_old;
            tf2::fromMsg(odom_in_camera_init.pose.pose.orientation, q_old);
            tf2::Quaternion q_delta;
            q_delta.setRPY(0, 0, delta_yaw);
            tf2::Quaternion q_new = q_old * q_delta;
            odom_in_camera_init.pose.pose.orientation = tf2::toMsg(q_new);

            // --- 协方差转换部分 huangxiaoshuai add 20250727---
            Eigen::Matrix<double, 6, 6> cov_in;      // 原始协方差
            Eigen::Matrix<double, 6, 6> cov_out;     // 旋转后协方差
            cov_in.setZero();

            // 拷贝原始协方差
            for (size_t i = 0; i < 36; ++i) {
                cov_in(i / 6, i % 6) = msg->pose.covariance[i];
            }

            // 从 transform 提取旋转矩阵
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3 rot(q);

            // 构建 6x6 旋转矩阵：上左3x3为rot，其余为单位阵
            Eigen::Matrix<double, 6, 6> rot6d = Eigen::Matrix<double, 6, 6>::Identity();
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    rot6d(r, c) = rot[r][c];
                    rot6d(r + 3, c + 3) = rot[r][c];
                }
            }

            // 协方差旋转
            cov_out = rot6d * cov_in * rot6d.transpose();

            // 写回 odom_in_camera_init
            for (size_t i = 0; i < 36; ++i) {
                odom_in_camera_init.pose.covariance[i] = cov_out(i / 6, i % 6);
            }
            // --- 协方差转换结束 huangxiaoshuai add 20250727---

            
            // 发布转换后的Odometry消息
            // RCLCPP_INFO(this->get_logger(), "发布utm_odom_in_camera_init消息");
            odom_pub_->publish(odom_in_camera_init);

            // 发布gps_init到gps的动态tf变换


            // transform = tf_buffer_->lookupTransform("gps_init", msg->header.frame_id, tf2::TimePointZero);
            transform = transform2;

            geometry_msgs::msg::PoseStamped pose_in_gps_init;
            tf2::doTransform(pose_in_odom, pose_in_gps_init, transform);

            // 3. 构造并广播 gps_init → gps 的动态 TF
            geometry_msgs::msg::TransformStamped gps_tf;
            gps_tf.header.stamp            = msg->header.stamp;
            gps_tf.header.frame_id         = "gps_init";
            gps_tf.child_frame_id          = "gps";
            gps_tf.transform.translation.x = pose_in_gps_init.pose.position.x;
            gps_tf.transform.translation.y = pose_in_gps_init.pose.position.y;
            // gps_tf.transform.translation.z = pose_in_gps_init.pose.position.z;
            gps_tf.transform.translation.z = 0.0;
            gps_tf.transform.rotation      = pose_in_gps_init.pose.orientation;

            // RCLCPP_INFO(this->get_logger(), "发布gps_init - gps的动态TF");
            tf_broadcaster_->sendTransform(gps_tf);
            

        }
        catch (tf2::TransformException &ex)
        {
            std::cout << "发布gps_init - gps出错" << std::endl;
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string input_odom_topic_;
    std::string output_topic;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
