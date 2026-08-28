#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "robot_interfaces/msg/zone_northp.hpp"
class OdomToGPSNode : public rclcpp::Node
{
public:
OdomToGPSNode()
        : Node("odom_to_gps_node"), has_get_UTM_zone(false)
    {

        

        // 获取body在utm/gps坐标系下的位姿
        this->declare_parameter<std::string>("fusion_odom_topic", "/odometry/filtered");
        this->declare_parameter<std::string>("target_frame", "utm");
        this->declare_parameter<std::string>("zone_northp_topic", "utm/zone/northp");   // 订阅utm地理分区，用于计算经纬度
        this->declare_parameter<std::string>("transformed_gps_topic", "gps/fusion");   // 发布转换后的GPS话题

        this->get_parameter("fusion_odom_topic", fusion_odom_topic);
        this->get_parameter("target_frame", target_frame);
        this->get_parameter("zone_northp_topic", zone_northp_topic);
        this->get_parameter("transformed_gps_topic", transformed_gps_topic);

        fusion_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(fusion_odom_topic, 10,
            std::bind(&OdomToGPSNode::fusion_odom_cb, this, std::placeholders::_1));
        zone_northp_sub_ = this->create_subscription<robot_interfaces::msg::ZoneNorthp>(zone_northp_topic, 10,
            std::bind(&OdomToGPSNode::UTM_zone_cb, this, std::placeholders::_1));

        transformed_gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(transformed_gps_topic, 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(),
                    "odom_to_GPS转换节点已启动, 订阅里程计话题: [%s], 转换并发布到话题：[%s]",
                    fusion_odom_topic.c_str(), transformed_gps_topic.c_str());
    }
private:
    std::string fusion_odom_topic, target_frame, zone_northp_topic, transformed_gps_topic;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fusion_odom_sub_;
    rclcpp::Subscription<robot_interfaces::msg::ZoneNorthp>::SharedPtr zone_northp_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr transformed_gps_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    int zone;
    bool northp, has_get_UTM_zone;

    // 读取UTM zone
    void UTM_zone_cb(const robot_interfaces::msg::ZoneNorthp::SharedPtr msg)
    {   
        if (!has_get_UTM_zone){
            zone = msg->zone;
            northp = msg->northp;
            RCLCPP_INFO(this->get_logger(), "读取UTM zone数据");
            has_get_UTM_zone = true;
        }
    }

    void fusion_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // std::string child_frame_id = msg->child_frame_id;
        // geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(target_frame, child_frame_id, rclcpp::Time(0), std::chrono::seconds(1));
        // RCLCPP_INFO(this->get_logger(), "Transform from %s to %s: %f, %f, %f",
        //             transform.header.frame_id.c_str(),
        //             transform.child_frame_id.c_str(),
        //             transform.transform.translation.x,
        //             transform.transform.translation.y,
        //             transform.transform.translation.z);
        // double x = transform.transform.translation.x;
        // double y = transform.transform.translation.y;
        // double z = transform.transform.translation.z;

        // msg是camera_init坐标系下的位姿，订阅camera_init对于utm的tf，计算出该位姿在utm下的表示
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        target_frame, msg->header.frame_id, rclcpp::Time(0), std::chrono::seconds(1));
        
        RCLCPP_INFO(this->get_logger(), "Transform from %s to %s: %f, %f, %f",
                    transform.header.frame_id.c_str(),
                    transform.child_frame_id.c_str(),
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z);

        // 取出msg中的位置
        double x_cam = msg->pose.pose.position.x;
        double y_cam = msg->pose.pose.position.y;
        double z_cam = msg->pose.pose.position.z;

        geometry_msgs::msg::PoseStamped point_in ,point_out;
        point_in.header = msg->header;
        point_in.pose = msg->pose.pose;
        tf2::doTransform(point_in, point_out, transform);

        // 反向转换为经纬度
        double lat, lon;
        GeographicLib::UTMUPS::Reverse(zone, northp, point_out.pose.position.x, point_out.pose.position.y, lat, lon);

        // 创建NavSatFix消息
        sensor_msgs::msg::NavSatFix gps_msg;
        gps_msg.header.stamp = this->now();
        gps_msg.header.frame_id = "gps";
        gps_msg.latitude = lat;
        gps_msg.longitude = lon;
        gps_msg.altitude = point_out.pose.position.z; // 假设高度为0，可以根据需要设置
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        // 发布消息
        // RCLCPP_INFO(this->get_logger(), "发布转换后的GPS数据 [经度：%.8f, 纬度：%.8f, 高度：%.3f]",
        //             gps_msg.longitude, gps_msg.latitude, gps_msg.altitude);
        transformed_gps_pub_->publish(gps_msg);
        
    }

};

/*
class OdomToGPSNode : public rclcpp::Node
{
public:
OdomToGPSNode()
        : Node("odom_to_gps_node"),has_get_init_pose(false), has_get_UTM_zone(false)
    {
        // 声明参数
        declare_parameter<std::string>("fusion_odom_topic", "odom_transformed");  // 输入的UTM里程计话题
        declare_parameter<std::string>("initial_pose_topic", "utm/initial_pose"); // 订阅初始定位姿态话题
        declare_parameter<std::string>("transformed_gps_topic", "gps/fusion");    // 转换后的GPS话题
        declare_parameter<std::string>("zone_northp_topic", "utm/zone/northp");   // 订阅utm地理分区，用于计算经纬度
        // 获取参数
        get_parameter("fusion_odom_topic", fusion_odom_topic);
        get_parameter("transformed_gps_topic", transformed_gps_topic);
        get_parameter("initial_pose_topic", initial_pose_topic);
        get_parameter("zone_northp_topic", zone_northp_topic);

        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(initial_pose_topic, 10,
            std::bind(&OdomToGPSNode::init_pose_cb, this, std::placeholders::_1));

        utm_zone_sub_ = this->create_subscription<robot_interfaces::msg::ZoneNorthp>(zone_northp_topic, 10,
            std::bind(&OdomToGPSNode::UTM_zone_cb, this, std::placeholders::_1));

        fusion_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(fusion_odom_topic, 10,
            std::bind(&OdomToGPSNode::fusion_odom_cb, this, std::placeholders::_1));

        transformed_gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(transformed_gps_topic, 10);

        RCLCPP_INFO(this->get_logger(),
                    "odom_to_GPS转换节点已启动, 订阅里程计话题: [%s], 转换并发布到话题：[%s]",
                    fusion_odom_topic.c_str(), transformed_gps_topic.c_str());
    
    }
private:
    // 读取UTM zone
    void UTM_zone_cb(const robot_interfaces::msg::ZoneNorthp::SharedPtr msg)
    {
        zone = msg->zone;
        northp = msg->northp;
        RCLCPP_INFO(this->get_logger(), "读取UTM zone数据");
        has_get_UTM_zone = true;
    }

    // 读取init pose
    void init_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        zero_x_ = msg->pose.position.x;
        zero_y_ = msg->pose.position.y;
        zero_z_ = msg->pose.position.z;
        RCLCPP_INFO(this->get_logger(), "读取init pose数据");
        has_get_init_pose = true;
    }
    
    // 订阅odom消息，转换成gps数据
    void fusion_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (has_get_init_pose && has_get_UTM_zone){
            double x = zero_x_ + msg->pose.pose.position.x;
            double y = zero_y_ + msg->pose.pose.position.y;
            double z = zero_z_ + msg->pose.pose.position.z;

            // 反向转换为经纬度
            double lat, lon;
            GeographicLib::UTMUPS::Reverse(zone, northp, x, y, lat, lon);

            // 创建NavSatFix消息
            sensor_msgs::msg::NavSatFix gps_msg;
            gps_msg.header.stamp = this->now();
            gps_msg.header.frame_id = "gps";
            gps_msg.latitude = lat;
            gps_msg.longitude = lon;
            gps_msg.altitude = z; // 假设高度为0，可以根据需要设置
            gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

            // 发布消息
            transformed_gps_pub_->publish(gps_msg);
        }
    }

    double zero_x_, zero_y_, zero_z_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fusion_odom_sub_;
    rclcpp::Subscription<robot_interfaces::msg::ZoneNorthp>::SharedPtr utm_zone_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr transformed_gps_pub_;
    
    // Parameters
    std::string fusion_odom_topic;
    std::string initial_pose_topic;
    std::string transformed_gps_topic;
    std::string zone_northp_topic;

    int zone;
    bool northp;
    bool has_get_init_pose;
    bool has_get_UTM_zone;
};
*/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToGPSNode>());
    rclcpp::shutdown();
    return 0;
}
