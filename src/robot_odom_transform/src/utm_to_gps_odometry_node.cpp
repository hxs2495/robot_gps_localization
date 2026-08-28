#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <mutex>
#include <vector>

class UtmZeroedOdometryNode : public rclcpp::Node
{
public:
    UtmZeroedOdometryNode()
        : Node("utm_zeroed_odometry_node"), collected_count_(0), zero_inited_(false), sum_x_(0.0), sum_y_(0.0), sum_z_(0.0)
    {
        // 声明参数
        declare_parameter<std::string>("input_odom_topic", "utm/gps_new"); // 输入的UTM里程计话题
        declare_parameter<std::string>("zeroed_odom_topic", "odometry/gps"); // 修正后的输出话题
        declare_parameter<std::string>("initial_pose_topic", "utm/initial_pose"); // 初始定位姿态话题
        declare_parameter<std::string>("zeroed_frame_id", "camera_init"); // 修正后的坐标系ID
        declare_parameter<bool>("publish_initial_pose", true); // 是否发布初始定位姿态
        declare_parameter<int>("init_frames", 10); // 初始化零点所需采集的帧数
        declare_parameter<std::string>("tf_pub_topic", "/odometry_tf");

        // 获取参数
        get_parameter("input_odom_topic", input_odom_topic_);
        get_parameter("zeroed_odom_topic", zeroed_odom_topic_);
        get_parameter("initial_pose_topic", initial_pose_topic_);
        get_parameter("zeroed_frame_id", zeroed_frame_id_);
        get_parameter("publish_initial_pose", is_pub_initial_pose_);
        get_parameter("init_frames", init_frames_);
        get_parameter("tf_pub_topic", tf_pub_topic_);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_, 10,
            std::bind(&UtmZeroedOdometryNode::odom_callback, this, std::placeholders::_1));

        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(initial_pose_topic_, 1);
        zeroed_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(zeroed_odom_topic_, 10);
        tf_pub_topic_ = this->get_parameter("tf_pub_topic").as_string();
		tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(tf_pub_topic_, 10);
        RCLCPP_INFO(this->get_logger(),
                    "UTM零点重定位节点已启动。采集前%d帧进行零点初始化。输入话题: [%s], 修正后输出话题: [%s], 初始定位话题: [%s]",
                    init_frames_, input_odom_topic_.c_str(), zeroed_odom_topic_.c_str(), initial_pose_topic_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!zero_inited_)
        {
            // 累加指定帧数
            sum_x_ += msg->pose.pose.position.x;
            sum_y_ += msg->pose.pose.position.y;
            sum_z_ += msg->pose.pose.position.z;
            ++collected_count_;

            RCLCPP_INFO(this->get_logger(), "采集初始零点第%d/%d帧: (%.3f, %.3f, %.3f)",
                        collected_count_, init_frames_,
                        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

            if (collected_count_ == init_frames_)
            {
                zero_x_ = sum_x_ / static_cast<double>(init_frames_);
                zero_y_ = sum_y_ / static_cast<double>(init_frames_);
                zero_z_ = sum_z_ / static_cast<double>(init_frames_);
                zero_inited_ = true;

                RCLCPP_INFO(this->get_logger(), "零点初始化完成: (%.3f, %.3f, %.3f)", zero_x_, zero_y_, zero_z_);
            }
            return;
        }

        // 只在第一次初始化后发布一次初始姿态
        // if (is_pub_initial_pose_ && !initial_pose_published_)
        // 一直发布初始位姿
        if (is_pub_initial_pose_)
        {
            geometry_msgs::msg::PoseStamped initial_pose;
            initial_pose.header.stamp = msg->header.stamp; // 保留原始消息的时间戳
            initial_pose.header.frame_id = msg->header.frame_id;
            initial_pose.pose.position.x = zero_x_;
            initial_pose.pose.position.y = zero_y_;
            initial_pose.pose.position.z = zero_z_;
            initial_pose.pose.orientation = msg->pose.pose.orientation;

            initial_pose_pub_->publish(initial_pose);
            initial_pose_published_ = true;
            RCLCPP_INFO(this->get_logger(), "发布初始定位姿态: (%.3f, %.3f, %.3f)",
                        zero_x_, zero_y_, zero_z_);
        }

        // 已初始化零点，发布修正后的里程计
        auto out = *msg;
        out.pose.pose.position.x -= zero_x_;
        out.pose.pose.position.y -= zero_y_;
        out.pose.pose.position.z -= zero_z_;
        out.header.frame_id = zeroed_frame_id_;
        zeroed_pub_->publish(out);
        RCLCPP_INFO(this->get_logger(), "发布零点修正后的里程计: (%.3f, %.3f, %.3f)",
                    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z);
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp; // 保留原始消息的时间戳
        tf_msg.header.frame_id = "camera_init";
        tf_msg.child_frame_id = "base_link";
        // translation: 使用最新的里程计位置
        tf_msg.transform.translation.x = msg->pose.pose.position.x;
        tf_msg.transform.translation.y = msg->pose.pose.position.y;
        tf_msg.transform.translation.z = msg->pose.pose.position.z;
        tf_msg.transform.rotation = msg->pose.pose.orientation; // fallback
        tf_pub_->publish(tf_msg);
        RCLCPP_INFO(this->get_logger(), "发布TF: camera_init -> base_link");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr zeroed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_pub_;

    std::mutex mutex_;
    int collected_count_;
    bool zero_inited_;
    bool initial_pose_published_ = false;
    double sum_x_, sum_y_, sum_z_;
    double zero_x_, zero_y_, zero_z_;
    // Parameters
    std::string input_odom_topic_;
    std::string zeroed_odom_topic_;
    std::string initial_pose_topic_;
    std::string zeroed_frame_id_;
    std::string tf_pub_topic_;
    bool is_pub_initial_pose_;
    int init_frames_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UtmZeroedOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
