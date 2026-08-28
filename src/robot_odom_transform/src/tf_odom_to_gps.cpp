#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>

class OdomTransformer : public rclcpp::Node
{
public:
    OdomTransformer() : Node("odom_transformer")
    {
        // 参数声明
        this->declare_parameter<std::string>("input_topic", "/Odometry");
        this->declare_parameter<std::string>("output_topic", "/Odometry/transform");
        
        // this->declare_parameter<double>("offset_x",-0.7016687);
        // this->declare_parameter<double>("offset_y", -0.18244671);
        
        this->declare_parameter<double>("offset_x",-0.84789221);
        this->declare_parameter<double>("offset_y", -0.21017876);
        this->declare_parameter<double>("offset_z", 0.0);
        this->declare_parameter<std::string>("output_frame", "camera_init");

        // 兼容旧的文本标定文件；留空时使用offset_*参数。
        this->declare_parameter<std::string>("ex_file_path", "");
        
        // 获取参数
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("offset_x", offset_x_);
        this->get_parameter("offset_y", offset_y_);
        this->get_parameter("offset_z", offset_z_);
        this->get_parameter("output_frame", output_frame_);

        this->get_parameter("ex_file_path", ex_file_path_);
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic_, 10,
            std::bind(&OdomTransformer::odom_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(),
                    "激光里程坐标装换节点已启动，订阅: %s，发布: %s，偏移量: [%.3f, %.3f, %.3f]，输出frame: %s",
                    input_topic_.c_str(), output_topic_.c_str(),
                    offset_x_, offset_y_, offset_z_, output_frame_.c_str());
        get_ex();
    }

private:

    void get_ex()
    {
        if (ex_file_path_.empty()) {
            RCLCPP_INFO(this->get_logger(), "未配置外参文件，使用offset_*默认标定参数");
            return;
        }
        std::ifstream fin(ex_file_path_);
        if (!fin.is_open()) {
            RCLCPP_WARN(this->get_logger(),
                "无法打开外参文件: %s，继续使用offset_*默认标定参数",
                ex_file_path_.c_str());
            return;
        } else {
            float x,y,z,qx,qy,qz,qw;
            if (!(fin >> x >> y >> z >> qx >> qy >> qz >> qw)) {
                RCLCPP_WARN(this->get_logger(),
                    "外参文件格式无效: %s，继续使用offset_*默认标定参数",
                    ex_file_path_.c_str());
                return;
            }
            fin.close();
            tf2::Quaternion q(qx, qy, qz, qw);
            tf2::Transform transform(q, tf2::Vector3(x, y, z));
            transform = transform.inverse();
            offset_x_ = transform.getOrigin().x();
            offset_y_ = transform.getOrigin().y();
            offset_z_ = transform.getOrigin().z();
            RCLCPP_INFO(this->get_logger(),
                    "get calib ex");
        }
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 提取原始位置和四元数
        const auto &pos = msg->pose.pose.position;
        const auto &quat = msg->pose.pose.orientation;

        // 四元数转旋转矩阵
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 rot(q);

        // 偏移向量在当前姿态下的表达
        tf2::Vector3 gps_in_body(offset_x_, offset_y_, offset_z_);
        tf2::Vector3 offset_in_camera = rot * gps_in_body;

        // 新的位置
        tf2::Vector3 pos_vec(pos.x, pos.y, pos.z);
        tf2::Vector3 new_pos = pos_vec + offset_in_camera;

        // 构造新的Odometry消息
        auto new_odom = std::make_shared<nav_msgs::msg::Odometry>(*msg);
        new_odom->header.frame_id = output_frame_;
        new_odom->pose.pose.position.x = new_pos.x();
        new_odom->pose.pose.position.y = new_pos.y();
        new_odom->pose.pose.position.z = 0.0;
        // 方向不变
        new_odom->pose.pose.orientation = quat;

        pub_->publish(*new_odom);
        // RCLCPP_INFO(this->get_logger(),
        //             "转换后的里程计数据已发布: [%.3f, %.3f, %.3f]",
        //             new_pos.x(), new_pos.y(), new_pos.z());
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    std::string input_topic_;
    std::string output_topic_;
    std::string output_frame_;
    std::string ex_file_path_;
    double offset_x_, offset_y_, offset_z_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
