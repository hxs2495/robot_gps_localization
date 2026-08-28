#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <filesystem>
#include <iomanip>

class PoseSaveNode : public rclcpp::Node
{
public:
  PoseSaveNode()
  : Node("pose_save")
  {
    this->declare_parameter<std::string>("ex_topic", "/ex_body2gps"); // GPS数据话题
    this->declare_parameter<std::string>("ex_file_path", "calib_ex.txt");
    this->declare_parameter<std::string>("ex_status_topic", "/ex_status_topic"); 
    auto ex_topic = this->get_parameter("ex_topic").as_string();
    ex_file_path_ = this->get_parameter("ex_file_path").as_string();
    ex_status_topic_ = this->get_parameter("ex_status_topic").as_string();
    ex_status_topic_pub_ = this->create_publisher<std_msgs::msg::String>(ex_status_topic_, 10);
        

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ex_topic, 10,
      std::bind(&PoseSaveNode::pose_cb, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PoseSaveNode::timer_cb, this));
  
    RCLCPP_INFO(get_logger(), "外参保存节点已启动！");
  }

  ~PoseSaveNode()
  {
    if (file_.is_open())
      file_.close();
  }

private:
  void timer_cb()
    {
      if (std::filesystem::exists(ex_file_path_)) {
        RCLCPP_INFO(get_logger(), "标定文件存在：%s", ex_file_path_.c_str());
        ex_status_topic_pub_->publish(std_msgs::msg::String().set__data("搜索到外参文件，正在读取..."));
        fin_.open(ex_file_path_, std::ios::in);
        if (fin_.is_open()) {
          float x,y,z,qx,qy,qz,qw;
          fin_ >> x >> y >> z >> qx >> qy >> qz >> qw;
          std_msgs::msg::String msg;
          msg.data = "读取成功，平移x:"+std::to_string(x)+" y:"+std::to_string(y)+" z:"+std::to_string(z)
                     +" 旋转qx:"+std::to_string(qx)+" qy:"+std::to_string(qy)+" qz:"+std::to_string(qz)+" qw:"+std::to_string(qw);
          ex_status_topic_pub_->publish(msg);
          fin_.close();
        }
      } else {
        RCLCPP_DEBUG(get_logger(), "文件不存在，继续检测 …");
      }
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto &p = msg->pose.position;
    const auto &q = msg->pose.orientation;
    
    file_.open(ex_file_path_,std::ios::trunc);
    file_ << p.x << ' ' << p.y << ' ' << p.z << ' '
          << q.x << ' ' << q.y << ' ' << q.z << ' ' << q.w << '\n';
    file_.close();   
    RCLCPP_INFO(get_logger(), "外参已保存！");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ex_status_topic_pub_;
  std::ofstream file_;
  std::ifstream fin_;
  std::string ex_file_path_, ex_status_topic_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSaveNode>());
  rclcpp::shutdown();
  return 0;
}
