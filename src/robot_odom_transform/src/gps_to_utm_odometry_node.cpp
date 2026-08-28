#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "robot_interfaces/msg/zone_northp.hpp"
#include <cmath> 
constexpr double RAD2DEG = 180.0 / M_PI;

class GpsToUtmOdomNode : public rclcpp::Node
{
public:
    GpsToUtmOdomNode()
        : Node("gps_to_utm_odometry_node"),
          last_easting_(0.0), last_northing_(0.0), last_orient_(0.0), last_stamp_(0)
    {
        this->declare_parameter<std::string>("gps_topic", "/fix/filter"); // GPS数据话题
        this->declare_parameter<std::string>("orient_topic", "/imu_orientation"); // IMU方向话题

        this->declare_parameter<std::string>("odom_topic", "utm/gps_new");    // 输出的UTM里程计话题
        this->declare_parameter<std::string>("zone_northp_topic", "utm/zone/northp");    // UTM分区和北半球标志话题
        this->declare_parameter<std::string>("frame_id", "utm");          // 坐标系ID
        this->declare_parameter<std::string>("child_frame_id", "gps");    // 子坐标系ID
        this->declare_parameter<bool>("use_velocity", false);             // 是否进行航向推算
        this->declare_parameter<bool>("use_yaw", false);                  // 是否进行速度推算
        this->declare_parameter<bool>("use_orientation", true);           // 是否使用外部航向
        this->declare_parameter<bool>("require_orientation", true);       // 首帧航向到达前是否等待
        this->declare_parameter<double>("orientation_variance", 0.04);     // 航向方差回退值(rad^2)
        this->declare_parameter<double>("unobserved_orientation_variance", 1.0e6);
        this->declare_parameter<double>("position_variance_floor", 0.01);
        this->declare_parameter<double>("unknown_position_variance", 4.0);

        auto gps_topic = this->get_parameter("gps_topic").as_string();
        auto orient_topic = this->get_parameter("orient_topic").as_string();
        auto odom_topic = this->get_parameter("odom_topic").as_string();
        auto zone_northp_topic = this->get_parameter("zone_northp_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        child_frame_id_ = this->get_parameter("child_frame_id").as_string();
        use_velocity_ = this->get_parameter("use_velocity").as_bool();
        use_yaw_ = this->get_parameter("use_yaw").as_bool();
        use_orientation_ = this->get_parameter("use_orientation").as_bool();
        require_orientation_ = this->get_parameter("require_orientation").as_bool();
        orientation_variance_ = this->get_parameter("orientation_variance").as_double();
        unobserved_orientation_variance_ =
            this->get_parameter("unobserved_orientation_variance").as_double();
        position_variance_floor_ = std::max(
            1.0e-9, this->get_parameter("position_variance_floor").as_double());
        unknown_position_variance_ = std::max(
            position_variance_floor_,
            this->get_parameter("unknown_position_variance").as_double());
        last_orientation_variance_ = orientation_variance_;

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic, rclcpp::SensorDataQoS(),
            std::bind(&GpsToUtmOdomNode::gps_callback, this, std::placeholders::_1));

        if (use_orientation_) {
            orient_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                orient_topic, rclcpp::SensorDataQoS(),
                std::bind(&GpsToUtmOdomNode::orient_callback, this, std::placeholders::_1));
        }

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
        zone_northp_pub_ = this->create_publisher<robot_interfaces::msg::ZoneNorthp>(zone_northp_topic, 10);

        RCLCPP_INFO(this->get_logger(), "GPS到UTM坐标转换节点已启动，订阅:%s 发布:%s", gps_topic.c_str(), odom_topic.c_str());
    }

private:

    void orient_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 从IMU消息中提取方向信息
        auto orientation_ = msg->orientation;
        const double norm_squared =
            orientation_.x * orientation_.x + orientation_.y * orientation_.y +
            orientation_.z * orientation_.z + orientation_.w * orientation_.w;
        if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "收到无效航向四元数，忽略该帧。");
            return;
        }
        // 转换成欧拉角，
        tf2::Quaternion quat(orientation_.x, orientation_.y, orientation_.z, orientation_.w);
        quat.normalize();
        tf2::Matrix3x3 m(quat);
        // 获取欧拉角
        double last_roll_, last_pitch_, last_heading_;
        m.getEulerYPR(last_heading_, last_pitch_, last_roll_);
        last_heading_ = last_heading_ * RAD2DEG;
        last_heading_ = 90-last_heading_;
        if (last_heading_ > 180)
        {
            last_heading_ -= 360.0; // 确保航向角在0-360度范围内
        }
        last_orient_ = last_heading_;
        const double message_variance = msg->orientation_covariance[8];
        last_orientation_variance_ =
            std::isfinite(message_variance) && message_variance > 0.0 ?
            message_variance : orientation_variance_;
        has_orientation_ = true;

    }
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // 只处理有效GPS数据
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "GPS无效，跳过此帧。");
            return;
        }
        if (use_orientation_ && require_orientation_ && !has_orientation_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000,
                "尚未收到航向数据，等待orient_topic后再发布UTM里程计。");
            return;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp; // 保留原始消息的时间戳
        odom_msg.header.frame_id = frame_id_;
        odom_msg.child_frame_id = child_frame_id_;

        double northing, easting;
        int zone;
        bool northp;
        try
        {
            GeographicLib::UTMUPS::Forward(
                msg->latitude, msg->longitude, zone, northp, easting, northing);

            odom_msg.pose.pose.position.x = easting;
            odom_msg.pose.pose.position.y = northing;
            odom_msg.pose.pose.position.z = msg->altitude;
            rclcpp::Time now = msg->header.stamp;

            // 方向推断：用前后两帧简单推算
            if (use_velocity_)
            {
                double vx = 0.0, vy = 0.0;
                if (last_stamp_.nanoseconds() > 0)
                {
                    double dt = (now - last_stamp_).seconds();
                    if (dt > 1e-3)
                    {
                        vx = (easting - last_easting_) / dt;
                        vy = (northing - last_northing_) / dt;
                    }
                }
                odom_msg.twist.twist.linear.x = vx;
                odom_msg.twist.twist.linear.y = vy;
            }

            // 欧拉角转换成四元数
            tf2::Quaternion quat;
            quat.setRPY(
                0, 0, use_orientation_ ? last_orient_ * M_PI / 180.0 : 0.0);
            odom_msg.pose.pose.orientation.x = quat.x();
            odom_msg.pose.pose.orientation.y = quat.y();
            odom_msg.pose.pose.orientation.z = quat.z();
            odom_msg.pose.pose.orientation.w = quat.w();

            // // 航向角估算
            // if (use_yaw_)
            // {
            //     // 计算航向角
            //     double yaw = 0.0;
            //     if (last_stamp_.nanoseconds() > 0)
            //     {
            //         double dx = easting - last_easting_;
            //         double dy = northing - last_northing_;
            //         if (dx != 0.0 || dy != 0.0)
            //         {
            //             yaw = std::atan2(dy, dx);
            //         }
            //     }
            //     odom_msg.pose.pose.orientation.x = 0.0;
            //     odom_msg.pose.pose.orientation.y = 0.0;
            //     odom_msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
            //     odom_msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
            // }
            // else
            // {
            //     // 如果不使用yaw，则设置为默认值
            //     odom_msg.pose.pose.orientation.x = 0.0;
            //     odom_msg.pose.pose.orientation.y = 0.0;
            //     odom_msg.pose.pose.orientation.z = 0.0;
            //     odom_msg.pose.pose.orientation.w = 1.0;
            // }

            // 协方差复制：将NavSatFix的3x3协方差矩阵拷贝到Odometry的6x6左上角。
            // 未知或全零协方差不能直接交给EKF，否则GPS会被当作近乎无噪声的绝对观测。
            for (size_t i = 0; i < 36; ++i)
            {
                odom_msg.pose.covariance[i] = 0.0;
            }
            const bool covariance_known =
                msg->position_covariance_type !=
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
            const bool horizontal_covariance_valid =
                std::isfinite(msg->position_covariance[0]) &&
                std::isfinite(msg->position_covariance[4]) &&
                msg->position_covariance[0] > 0.0 &&
                msg->position_covariance[4] > 0.0;

            if (covariance_known && horizontal_covariance_valid)
            {
                for (size_t row = 0; row < 3; ++row)
                {
                    for (size_t col = 0; col < 3; ++col)
                    {
                        const double covariance = msg->position_covariance[row * 3 + col];
                        odom_msg.pose.covariance[row * 6 + col] =
                            std::isfinite(covariance) ? covariance : 0.0;
                    }
                }
                odom_msg.pose.covariance[0] =
                    std::max(position_variance_floor_, odom_msg.pose.covariance[0]);
                odom_msg.pose.covariance[7] =
                    std::max(position_variance_floor_, odom_msg.pose.covariance[7]);
                odom_msg.pose.covariance[14] =
                    std::max(position_variance_floor_, odom_msg.pose.covariance[14]);
            }
            else
            {
                odom_msg.pose.covariance[0] = unknown_position_variance_;
                odom_msg.pose.covariance[7] = unknown_position_variance_;
                odom_msg.pose.covariance[14] = unknown_position_variance_;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "GPS位置协方差未知或无效，使用回退方差 %.3f m^2",
                    unknown_position_variance_);
            }
            // NavSatFix没有姿态协方差。roll/pitch不观测；yaw使用航向消息或参数方差。
            odom_msg.pose.covariance[21] = unobserved_orientation_variance_;
            odom_msg.pose.covariance[28] = unobserved_orientation_variance_;
            odom_msg.pose.covariance[35] =
                use_orientation_ ? last_orientation_variance_ :
                unobserved_orientation_variance_;
            // 速度协方差也可简单设置
            // odom_msg.twist.covariance[0] = 0.2;
            // odom_msg.twist.covariance[7] = 0.2;
            // odom_msg.twist.covariance[14] = 0.2;

            // 增加zone信息，可在frame_id或child_frame_id中体现，或发布diagnostic
            // odom_msg.header.frame_id = frame_id_ + "_zone" + std::to_string(zone) + (northp ? "N" : "S");
            odom_msg.header.frame_id = frame_id_;

            odom_pub_->publish(odom_msg);
            publishZoneNorthp(zone, northp);
            // 保存历史
            last_easting_ = easting;
            last_northing_ = northing;
            last_stamp_ = now;

            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            //                      "GPS转换UTM: zone %d %s (%.2f, %.2f), yaw: %.2f deg, vx: %.2f m/s, vy: %.2f m/s",
            //                      zone, northp ? "N" : "S", easting, northing, yaw * 180.0 / M_PI, vx, vy);

            // RCLCPP_INFO(this->get_logger(), "GPS转换UTM: zone %d %s (%.2f, %.2f), yaw: %.2f deg",
            //             zone, northp ? "N" : "S", easting, northing, last_orient_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "UTM转换失败: %s", e.what());
        }
    }
    void publishZoneNorthp(int zone, bool northp)
    {
        auto zone_northp_msg = robot_interfaces::msg::ZoneNorthp();
        zone_northp_msg.zone = zone;
        zone_northp_msg.northp = northp;
        zone_northp_pub_->publish(zone_northp_msg);
    }
    std::string frame_id_, child_frame_id_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orient_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<robot_interfaces::msg::ZoneNorthp>::SharedPtr zone_northp_pub_;

    // 用于速度和航向估算
    double last_easting_, last_northing_, last_orient_;
    rclcpp::Time last_stamp_;
    bool use_velocity_, use_yaw_;
    bool use_orientation_, require_orientation_, has_orientation_{false};
    double orientation_variance_, unobserved_orientation_variance_;
    double position_variance_floor_, unknown_position_variance_;
    double last_orientation_variance_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsToUtmOdomNode>());
    rclcpp::shutdown();
    return 0;
}
