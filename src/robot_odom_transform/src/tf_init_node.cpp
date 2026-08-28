#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <tf2/utils.h>
#include <cmath> 
constexpr double RAD2DEG = 180.0 / M_PI;
class TFInitNode : public rclcpp::Node
{
public:
    TFInitNode()
        : Node("tf_init_node"), collected_count_(0), sum_x_(0.0), sum_y_(0.0), sum_z_(0.0),
        sum_orient_x_(0.0), sum_orient_y_(0.0), sum_orient_z_(0.0), sum_orient_w_(0.0), 
        zero_x_(0.0), zero_y_(0.0), zero_z_(0.0), 
        zero_orient_x_(0.0), zero_orient_y_(0.0), zero_orient_z_(0.0), zero_orient_w_(0.0),
        has_pub_tf_(false), odom_zeroed_(false)
    {
        // 声明参数
        declare_parameter<std::string>("input_odom_topic", "utm/gps_new"); // 输入的UTM里程计话题
        declare_parameter<int>("init_frames", 10); // 初始化零点所需采集的帧数
        declare_parameter<std::string>("output_topic", "/odom_in_map");
        declare_parameter<double>("ex_x", 0.726732);
        declare_parameter<double>("ex_y", -0.109451);
        declare_parameter<double>("ex_z", 0.0);
        declare_parameter<double>("ex_orient_w", 0.966142);
        declare_parameter<double>("ex_orient_x", 0.00259991);
        declare_parameter<double>("ex_orient_y", 0.00764529);
        declare_parameter<double>("ex_orient_z", -0.257884);
        
        // 兼容旧的文本标定文件；留空时直接使用上面的ROS参数默认值。
        declare_parameter<std::string>("ex_file_path", "");
        // 获取参数

        get_parameter("input_odom_topic", input_odom_topic_);
        get_parameter("init_frames", init_frames_);
        get_parameter("output_topic", output_topic);
        // 李佳宾 9.4 从launch传入外参
        get_parameter("ex_x", ex_x_);
        get_parameter("ex_y", ex_y_);
        get_parameter("ex_z", ex_z_);
        get_parameter("ex_orient_x", ex_orient_x_);
        get_parameter("ex_orient_y", ex_orient_y_);
        get_parameter("ex_orient_z", ex_orient_z_);
        get_parameter("ex_orient_w", ex_orient_w_);

        get_parameter("ex_file_path", ex_file_path_);
        
        // 初始化tf缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_, 10,
            std::bind(&TFInitNode::odom_callback, this, std::placeholders::_1));

        // 李佳宾 25/8/10 已在gps_to_utm_odometry_node中转换，直接读取utm/gps的旋转即可
        // orient_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(orient_topic_, 10,
        //     std::bind(&TFInitNode::orient_callback, this, std::placeholders::_1));
        
        // reset_fastlio_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        //     "/reset_fastlio", 10, std::bind(&TFInitNode::reset_fastlio_callback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
        
        RCLCPP_INFO(this->get_logger(),
                    "TF初始化节点已启动。采集前%d帧进行零点初始化。输入话题: [%s]",
                    init_frames_, input_odom_topic_.c_str());
        get_ex();
    }

private:

    /*
    // 李佳宾 9.23 不再使用直接读取文件
    void ex_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 处理外参消息
        ex_x_ = msg->pose.position.x;
        ex_y_ = msg->pose.position.y;
        ex_z_ = msg->pose.position.z;

        ex_orient_x_ = msg->pose.orientation.x;
        ex_orient_y_ = msg->pose.orientation.y;
        ex_orient_z_ = msg->pose.orientation.z;
        ex_orient_w_ = msg->pose.orientation.w;

        RCLCPP_INFO(this->get_logger(), "外参已接收: (%.3f, %.3f, %.3f), 方向: (%.3f, %.3f, %.3f, %.3f)",
                    ex_x_, ex_y_, ex_z_,
                    ex_orient_x_, ex_orient_y_, ex_orient_z_, ex_orient_w_);
        if (!has_pub_tf_){
            RCLCPP_INFO(this->get_logger(), "等待固定gps_init");
        }else if (!has_get_ex_)
        {
            // 只处理一次
            geometry_msgs::msg::TransformStamped transformStamped;
            //发布gps到body的坐标系转换
            transformStamped.header.stamp = test_stamp;
            transformStamped.header.frame_id = "gps";
            transformStamped.child_frame_id = "gps_body";

            transformStamped.transform.translation.x = ex_x_;
            transformStamped.transform.translation.y = ex_y_;
            transformStamped.transform.translation.z = ex_z_; 

            transformStamped.transform.rotation.x = ex_orient_x_;
            transformStamped.transform.rotation.y = ex_orient_y_;
            transformStamped.transform.rotation.z = ex_orient_z_;
            transformStamped.transform.rotation.w = ex_orient_w_;

            tf_broadcaster_->sendTransform(transformStamped);

            //发布gps_init到camera_init的坐标系转换
            transformStamped.header.stamp = test_stamp;
            transformStamped.header.frame_id = "gps_init";
            transformStamped.child_frame_id = "camera_init";

            tf_broadcaster_->sendTransform(transformStamped);

            RCLCPP_INFO(this->get_logger(), "发布gps到body的坐标系转换关系");
            has_get_ex_ = true;
        }
    }
    */
    void get_ex(){
        if (ex_file_path_.empty()) {
            RCLCPP_INFO(this->get_logger(), "未配置外参文件，使用ROS参数中的默认标定");
            has_get_ex_ = true;
            return;
        }
        std::ifstream fin(ex_file_path_, std::ios::in);
        if (!fin.is_open()) {
            RCLCPP_WARN(this->get_logger(),
                "无法打开外参文件: %s，继续使用ROS参数中的默认标定",
                ex_file_path_.c_str());
            has_get_ex_ = true;
            return;
        } else {
            double x, y, z, qx, qy, qz, qw;
            if (!(fin >> x >> y >> z >> qx >> qy >> qz >> qw)) {
                RCLCPP_WARN(this->get_logger(),
                    "外参文件格式无效: %s，继续使用ROS参数中的默认标定",
                    ex_file_path_.c_str());
                has_get_ex_ = true;
                return;
            }
            fin.close();
            ex_x_ = x;
            ex_y_ = y;
            ex_z_ = z;
            ex_orient_x_ = qx;
            ex_orient_y_ = qy;
            ex_orient_z_ = qz;
            ex_orient_w_ = qw;
            RCLCPP_INFO(this->get_logger(), "外参已读取: (%.3f, %.3f, %.3f), 方向: (%.3f, %.3f, %.3f, %.3f)",
                        ex_x_, ex_y_, ex_z_,
                        ex_orient_x_, ex_orient_y_, ex_orient_z_, ex_orient_w_);
            has_get_ex_ = true;
        }
    }
    // 李佳宾 25/8/13 已在fast_lio中处理
    /*
    void reset_fastlio_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Resetting FastLIO...");
            // 获取gps_body到gps_init的变换
            geometry_msgs::msg::TransformStamped transform;
            try {   
                transform = tf_buffer_->lookupTransform("gps_init", "gps_body", rclcpp::Time(0));
                // 打印transform信息
                RCLCPP_INFO(this->get_logger(), "Transform from %s to %s: %f, %f, %f, %f, %f, %f, %f",
                            transform.header.frame_id.c_str(),
                            transform.child_frame_id.c_str(),
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z,
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w);
                // 更新camera_init到gps_init的变换
                geometry_msgs::msg::TransformStamped camera_init_to_gps_init;
                camera_init_to_gps_init.header.stamp = this->now();
                camera_init_to_gps_init.header.frame_id = "gps_init";
                camera_init_to_gps_init.child_frame_id = "camera_init";
                camera_init_to_gps_init.transform.translation.x = transform.transform.translation.x;
                camera_init_to_gps_init.transform.translation.y = transform.transform.translation.y;
                camera_init_to_gps_init.transform.translation.z = transform.transform.translation.z;
                camera_init_to_gps_init.transform.rotation = transform.transform.rotation;
                // tf_broadcaster_->sendTransform(camera_init_to_gps_init);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Could not transform gps_init to gps_body: %s", ex.what());
                return;
            }
        }
    }
    */

    // 李佳宾 25/8/10 已在gps_to_utm_odometry_node中转换，直接读取utm/gps的朝向即可
    /*
    void orient_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 从imu消息中提取方向信息
        if (orient_count_ >= init_frames_)
        {
            zero_orient_ = sum_orient_ / static_cast<double>(orient_count_);
            //弧度转角度值
            zero_orient_ = zero_orient_*RAD2DEG;
            
            double offset_axis_z = 90-zero_orient_;
            if (offset_axis_z > 180)
            {
                offset_axis_z -= 360;
            }
            RCLCPP_INFO(this->get_logger(), "gps朝向初始化完成: %.3f", offset_axis_z);

            if (!has_pub_tf && odom_zeroed_){
                // 发布utm到gps的坐标系转换
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = msg->header.stamp;
                transformStamped.header.frame_id = frame_id_;
                transformStamped.child_frame_id = "gps_init";
                transformStamped.transform.translation.x = zero_x_;
                transformStamped.transform.translation.y = zero_y_;
                transformStamped.transform.translation.z = zero_z_;
                
                tf2::Quaternion q;
                q.setRPY(0, 0, offset_axis_z * M_PI / 180.0); // 将角度转换为弧度
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(transformStamped);
                RCLCPP_INFO(this->get_logger(), "发布utm到gps的坐标系转换: (%.3f, %.3f, %.3f), 方向: %.3f",
                            zero_x_, zero_y_, zero_z_, offset_axis_z);

                //发布gps到body的坐标系转换
                transformStamped.header.stamp = msg->header.stamp;
                transformStamped.header.frame_id = "gps";
                transformStamped.child_frame_id = "gps_body";

                transformStamped.transform.translation.x = 0.696864;
                transformStamped.transform.translation.y = -0.203795;
                transformStamped.transform.translation.z = 0.0; 

                transformStamped.transform.rotation.x = 0.00953811;
                transformStamped.transform.rotation.y = 0.00411306;
                transformStamped.transform.rotation.z = -0.26463;
                transformStamped.transform.rotation.w = 0.964294;

                

                tf_broadcaster_->sendTransform(transformStamped);

                //发布gps_init到camera_init的坐标系转换
                transformStamped.header.stamp = msg->header.stamp;
                transformStamped.header.frame_id = "gps_init";
                transformStamped.child_frame_id = "camera_init";

                transformStamped.transform.translation.x = 0.696864;
                transformStamped.transform.translation.y = -0.203795;
                transformStamped.transform.translation.z = 0.0; 
                
                transformStamped.transform.rotation.x = 0.00953811;
                transformStamped.transform.rotation.y = 0.00411306;
                transformStamped.transform.rotation.z = -0.26463;
                transformStamped.transform.rotation.w = 0.964294;
                
                
                tf_broadcaster_->sendTransform(transformStamped);
                has_pub_tf = true;

                RCLCPP_INFO(this->get_logger(), "发布gps到body的坐标系转换关系");
            }
            return;
        }
        //从imu消息中获取四元数，并转化成欧拉角
        double roll, pitch, yaw;
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        // 累加方向信息
        RCLCPP_INFO(this->get_logger(), "IMU方向: (%.3f, %.3f, %.3f)", roll, pitch, yaw);

        sum_orient_ += yaw; 
        ++orient_count_;
    }
    */
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        if (collected_count_ >= init_frames_)
        {   
            // 打印has_pub_tf和odom_zeroed_的值
            // RCLCPP_INFO(this->get_logger(), "has_pub_tf: %s, odom_zeroed_: %s",
            //             has_pub_tf_ ? "true" : "false",
            //             odom_zeroed_ ? "true" : "false");
            
            // 在已经计算计算起始点的情况下，只发布一次tf静态关系
            // 计算零点,只计算一次
            if (!odom_zeroed_) {

                zero_x_ = sum_x_ / static_cast<double>(init_frames_);
                zero_y_ = sum_y_ / static_cast<double>(init_frames_);
                zero_z_ = sum_z_ / static_cast<double>(init_frames_);
                
                zero_orient_x_ = sum_orient_x_ / static_cast<double>(init_frames_);
                zero_orient_y_ = sum_orient_y_ / static_cast<double>(init_frames_);
                zero_orient_z_ = sum_orient_z_ / static_cast<double>(init_frames_);
                zero_orient_w_ = sum_orient_w_ / static_cast<double>(init_frames_);
                
                frame_id_ = msg->header.frame_id;
                RCLCPP_INFO(this->get_logger(), "零点已初始化完成: (%.3f, %.3f, %.3f)", zero_x_, zero_y_, zero_z_);
                odom_zeroed_ = true;
            }else if (!has_pub_tf_ && has_get_ex_) {
                
                test_stamp = msg->header.stamp;
                // 发布utm到gps的坐标系转换
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = test_stamp;
                transformStamped.header.frame_id = frame_id_;
                transformStamped.child_frame_id = "gps_init";
                transformStamped.transform.translation.x = zero_x_;
                transformStamped.transform.translation.y = zero_y_;
                transformStamped.transform.translation.z = zero_z_;

                transformStamped.transform.rotation.x = zero_orient_x_;
                transformStamped.transform.rotation.y = zero_orient_y_;
                transformStamped.transform.rotation.z = zero_orient_z_;
                transformStamped.transform.rotation.w = zero_orient_w_;

                tf_broadcaster_->sendTransform(transformStamped);
                RCLCPP_INFO(this->get_logger(), "发布utm到gps的坐标系转换: (%.3f, %.3f, %.3f)",
                            zero_x_, zero_y_, zero_z_);

                // 李佳宾 9.4 从ex_callback中复制，通过launch配置外参，从而取消pub_ex节点 
                //发布gps到body的坐标系转换
                transformStamped.header.stamp = test_stamp;
                transformStamped.header.frame_id = "gps";
                transformStamped.child_frame_id = "gps_body";

                transformStamped.transform.translation.x = ex_x_;
                transformStamped.transform.translation.y = ex_y_;
                transformStamped.transform.translation.z = ex_z_; 

                transformStamped.transform.rotation.x = ex_orient_x_;
                transformStamped.transform.rotation.y = ex_orient_y_;
                transformStamped.transform.rotation.z = ex_orient_z_;
                transformStamped.transform.rotation.w = ex_orient_w_;

                tf_broadcaster_->sendTransform(transformStamped);

                //发布gps_init到camera_init的坐标系转换
                transformStamped.header.stamp = test_stamp;
                transformStamped.header.frame_id = "gps_init";
                transformStamped.child_frame_id = "camera_init";

                tf_broadcaster_->sendTransform(transformStamped);

                RCLCPP_INFO(this->get_logger(), "发布gps到body的坐标系转换关系");

                has_pub_tf_= true;
            }
            // 李佳宾 8/23
            // odom_zeroed_ 为 true has_pub_tf_ == true
            else{
                RCLCPP_INFO(this->get_logger(),"坐标系转换");
                // 李佳宾 8/29 从odom_repub_with_tf.cpp中复制，用于去掉从odom_repub_with_tf
                // 具体功能
                // 1. 获取camera_init 到 utm的坐标系转换，将utm/gps消息的位置转换到camera_init下，并转换协方差用于融合
                // 2. 订阅gps到gps_body的外参，用于转换utm/gps消息的朝向，以进行后续融合
                // 3. 订阅gps_init到utm的坐标系转换，获取utm/gps消息在gps_init下的观测，用于发布gps_init到gps的动态tf

                // 李佳宾 8/23 copy from odom_repub_with_tf.cpp
                try
                {
                    // 获取从UTM到camera_init的tf转换
                    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("camera_init", msg->header.frame_id, tf2::TimePointZero);
                    // geometry_msgs::msg::TransformStamped tf_body2gps = tf_buffer_->lookupTransform("gps", "gps_body", tf2::TimePointZero);
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
                    
                    // 李佳宾 9.3 从成员变量中获取外参，从而取消tf_body2gps的查找
                    tf2::Quaternion q;
                    q.setX(ex_orient_x_);
                    q.setY(ex_orient_y_);
                    q.setZ(ex_orient_z_);
                    q.setW(ex_orient_w_);
                    // lijiabin 25/8/12  此前朝向为天线朝向在camera_init坐标系下的观测，其本身依然是天线的朝向
                    //                   订阅body到gps旋转来转换成车体朝向
                    // double delta_yaw = tf2::getYaw(tf_body2gps.transform.rotation);
                    double delta_yaw = tf2::getYaw(q);
                    // RCLCPP_INFO(this->get_logger(), "body到GPS的偏航角: %.2f", delta_yaw);
                    tf2::Quaternion q_old;
                    tf2::fromMsg(odom_in_camera_init.pose.pose.orientation, q_old);
                    tf2::Quaternion q_delta;
                    q_delta.setRPY(0, 0, delta_yaw);
                    // tf2::Quaternion q_new = q_delta * q_old;
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
                    q.setX(transform.transform.rotation.x);
                    q.setY(transform.transform.rotation.y);
                    q.setZ(transform.transform.rotation.z);
                    q.setW(transform.transform.rotation.w);
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
        }else{
        // 累加指定帧数
        sum_x_ += msg->pose.pose.position.x;
        sum_y_ += msg->pose.pose.position.y;
        sum_z_ += msg->pose.pose.position.z;
        sum_orient_x_ += msg->pose.pose.orientation.x;
        sum_orient_y_ += msg->pose.pose.orientation.y;
        sum_orient_z_ += msg->pose.pose.orientation.z; // 假设使用z轴方向作为朝向
        sum_orient_w_ += msg->pose.pose.orientation.w;
        
        ++collected_count_;
        
        RCLCPP_INFO(this->get_logger(), "采集初始零点第%d/%d帧: (%.3f, %.3f, %.3f)",
        collected_count_, init_frames_,
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }
}


    rclcpp::Time test_stamp;



    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orient_sub_;
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_fastlio_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double sum_x_, sum_y_, sum_z_, sum_orient_x_, sum_orient_y_, sum_orient_z_, sum_orient_w_;
    int collected_count_;

    double zero_x_, zero_y_, zero_z_, zero_orient_x_, zero_orient_y_, zero_orient_z_, zero_orient_w_;
    bool has_pub_tf_ = false, odom_zeroed_ = false, has_get_ex_ = false;

    double ex_x_, ex_y_, ex_z_, ex_orient_x_, ex_orient_y_, ex_orient_z_, ex_orient_w_;
    // Parameters
    std::string frame_id_;
    std::string input_odom_topic_,output_topic;
    std::string ex_file_path_;

    int init_frames_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFInitNode>());
    rclcpp::shutdown();
    return 0;
}
