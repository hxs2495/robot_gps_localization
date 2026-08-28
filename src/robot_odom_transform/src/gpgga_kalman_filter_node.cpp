#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <cmath>

using std::placeholders::_1;

/* ============================== */
/* ===  新增：一维卡尔曼滤波器 === */
/* ============================== */
class Kalman1D
{
public:
    Kalman1D(double Q, double R)
        : Q_(Q), R_(R), x_(0.0), P_(1e6), initialized_(false) {}

    void reset()
    {
        initialized_ = false;
        P_ = 1e6;
    }

    double update(double measurement)
    {
        if (!initialized_)
        {
            x_ = measurement;
            initialized_ = true;
            return x_;
        }

        // Time update
        P_ += Q_;

        // Measurement update
        double K = P_ / (P_ + R_);
        x_ = x_ + K * (measurement - x_);
        P_ = (1 - K) * P_;

        return x_;
    }

private:
    double Q_;   // process noise
    double R_;   // measurement noise
    double x_;   // state
    double P_;   // covariance
    bool initialized_;
};

class NavSatFixFilterNode : public rclcpp::Node
{
public:
    NavSatFixFilterNode()
    : Node("navsatfix_filter_node"),
      data_active_(false),
      consecutive_valid_count_(0)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&NavSatFixFilterNode::topic_callback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "fix/filter", 10);

        this->declare_parameter<int>("min_sats", 15);
        this->declare_parameter<int>("required_status", 2);
        this->declare_parameter<int>("recover_frame_count", 100);

        // === 新增：KF 参数 ===
        this->declare_parameter<double>("kf_process_noise", 1e-6);
        this->declare_parameter<double>("kf_measure_noise", 1e-3);

        double Q = this->get_parameter("kf_process_noise").as_double();
        double R = this->get_parameter("kf_measure_noise").as_double();

        kf_lat_ = std::make_shared<Kalman1D>(Q, R);
        kf_lon_ = std::make_shared<Kalman1D>(Q, R);
        kf_alt_ = std::make_shared<Kalman1D>(Q, R);

        RCLCPP_INFO(this->get_logger(), "NavSatFix 过滤节点 + 卡尔曼滤波 已启动");
    }

private:
    void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        int min_sats = this->get_parameter("min_sats").as_int();
        int required_status = this->get_parameter("required_status").as_int();
        int recover_frame_count = this->get_parameter("recover_frame_count").as_int();

        bool valid = (msg->status.status >= required_status &&
                      static_cast<int>(msg->status.service) >= min_sats);

        if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude))
        {
            RCLCPP_WARN(this->get_logger(), "无效坐标，丢弃");
            mark_data_invalid();
            return;
        }

        double cov_lat = msg->position_covariance[0];
        double cov_lon = msg->position_covariance[4];
        if (cov_lat < 0 || cov_lon < 0)
        {
            RCLCPP_WARN(this->get_logger(), "无效协方差");
            mark_data_invalid();
            return;
        }

        // 状态恢复逻辑
        if (valid)
        {
            consecutive_valid_count_++;
            if (!data_active_ && consecutive_valid_count_ >= recover_frame_count)
            {
                data_active_ = true;

                // === 新增：KF 重置 ===
                kf_lat_->reset();
                kf_lon_->reset();
                kf_alt_->reset();

                RCLCPP_INFO(this->get_logger(), "GPS 数据稳定恢复，启动卡尔曼滤波");
            }
        }
        else
        {
            if (data_active_)
                RCLCPP_WARN(this->get_logger(), "GPS 数据失效，暂停发布");
            mark_data_invalid();
        }

        if (!data_active_)
            return;

        /* ========================================== */
        /* ===        新增：卡尔曼滤波处理         === */
        /* ========================================== */
        double f_lat = kf_lat_->update(msg->latitude);
        double f_lon = kf_lon_->update(msg->longitude);
        double f_alt = kf_alt_->update(msg->altitude);

        sensor_msgs::msg::NavSatFix filtered_msg = *msg;
        filtered_msg.latitude = f_lat;
        filtered_msg.longitude = f_lon;
        filtered_msg.altitude = f_alt;

        publisher_->publish(filtered_msg);

        RCLCPP_INFO(this->get_logger(),
            "发布 KF 平滑后数据: lat=%.8f lon=%.8f alt=%.3f",
            f_lat, f_lon, f_alt);
    }

    void mark_data_invalid()
    {
        data_active_ = false;
        consecutive_valid_count_ = 0;

        kf_lat_->reset();
        kf_lon_->reset();
        kf_alt_->reset();
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;

    bool data_active_;
    int consecutive_valid_count_;

    // === 新增：三轴 KF ===
    std::shared_ptr<Kalman1D> kf_lat_;
    std::shared_ptr<Kalman1D> kf_lon_;
    std::shared_ptr<Kalman1D> kf_alt_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavSatFixFilterNode>());
    rclcpp::shutdown();
    return 0;
}
