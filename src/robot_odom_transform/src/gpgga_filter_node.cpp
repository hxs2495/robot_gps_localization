#include <algorithm>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;

class NavSatFixFilterNode : public rclcpp::Node
{
public:
    NavSatFixFilterNode()
        : Node("navsatfix_filter_node"),
          data_active_(false),
          consecutive_valid_count_(0)
    {
        this->declare_parameter<std::string>("input_topic", "/fix");
        this->declare_parameter<std::string>("output_topic", "/fix/filter");
        this->declare_parameter<int>("min_sats", 15);             // 最少卫星数
        this->declare_parameter<int>("required_status", 0);       // 定位状态 (例如 RTK Fixed)
        this->declare_parameter<int>("recover_frame_count", 100); // 连续多少帧恢复

        const auto input_topic = this->get_parameter("input_topic").as_string();
        const auto output_topic = this->get_parameter("output_topic").as_string();

        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&NavSatFixFilterNode::topic_callback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            output_topic, rclcpp::SensorDataQoS());

        RCLCPP_INFO(
            this->get_logger(), "NavSatFix过滤节点已启动: %s -> %s",
            input_topic.c_str(), output_topic.c_str());
    }

private:
    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        int required_status = this->get_parameter("required_status").as_int();
        int recover_frame_count = std::max(
            1, static_cast<int>(
                this->get_parameter("recover_frame_count").as_int()));

        // 当前数据是否符合过滤条件
        // bool valid = (msg->status.status >= required_status &&
        //               static_cast<int>(msg->status.service) >= min_sats);

        // 打印状态valid
        // RCLCPP_INFO(this->get_logger(), "当前状态valid: %d", valid);

        // Step 1: 检查经纬度和高度是否有效
        if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude))
        {
            RCLCPP_WARN(this->get_logger(), "收到无效坐标数据，丢弃");
            mark_data_invalid();
            return;
        }

        if (msg->status.status < required_status)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "GPS定位状态不足，当前=%d，要求>=%d",
                msg->status.status, required_status);
            mark_data_invalid();
            return;
        }

        // Step 2: 检查协方差
        double cov_lat = msg->position_covariance[0];
        double cov_lon = msg->position_covariance[4];
        if (cov_lat < 0 || cov_lon < 0)
        {
            RCLCPP_WARN(this->get_logger(), "收到无效协方差，纬度方差=%.3f，经度方差=%.3f", cov_lat, cov_lon);
            mark_data_invalid();
            return;
        }

        // Step 3: 根据有效性计数
        // if (valid)
        // {
        //     consecutive_valid_count_++;
        //     if (!data_active_ && consecutive_valid_count_ >= recover_frame_count)
        //     {
        //         data_active_ = true;
        //         RCLCPP_INFO(this->get_logger(),
        //             "GPS 数据已稳定恢复 (%d/%d 连续有效帧)",
        //             consecutive_valid_count_, recover_frame_count);
        //     }
        //     else if (!data_active_)
        //     {
        //         RCLCPP_INFO(this->get_logger(),
        //             "GPS 数据恢复进度: %d/%d",
        //             consecutive_valid_count_, recover_frame_count);
        //     }
        // }
        // else
        // {
        //     if (data_active_)
        //     {
        //         RCLCPP_WARN(this->get_logger(), "GPS 数据失效，立即关闭发布");
        //     }
        //     mark_data_invalid();
        // }

        // Step 4: 发布数据（仅在稳定状态下）
        // if (data_active_)
        // {
        //     publisher_->publish(*msg);
        //     RCLCPP_INFO(this->get_logger(),
        //         "发布过滤后的 NavSatFix: 状态=%d, 卫星数=%d, 海拔=%.2f",
        //         msg->status.status, static_cast<int>(msg->status.service), msg->altitude);
        // }

        ++consecutive_valid_count_;
        if (consecutive_valid_count_ < recover_frame_count)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "等待GPS稳定: %d/%d", consecutive_valid_count_, recover_frame_count);
            return;
        }

        data_active_ = true;
        publisher_->publish(*msg);
        RCLCPP_DEBUG(this->get_logger(),
                     "发布过滤后的NavSatFix: 状态=%d, service=%d, 海拔=%.2f",
                     msg->status.status, static_cast<int>(msg->status.service), msg->altitude);
    }

    void mark_data_invalid()
    {
        data_active_ = false;
        consecutive_valid_count_ = 0;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

    bool data_active_;            // 数据是否稳定可用
    int consecutive_valid_count_; // 连续有效帧计数
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavSatFixFilterNode>());
    rclcpp::shutdown();
    return 0;
}
