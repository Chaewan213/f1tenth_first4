// safety_filter_node.cpp
// /drive_raw -> [유효성 검사, AEB, 워치독] -> /drive
// 평소에는 명령을 그대로 통과시킨다. 회피 조향·속도 계획은 하지 않는다.
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "race_control/safety_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using sensor_msgs::msg::LaserScan;

class SafetyFilterNode : public rclcpp::Node
{
public:
  SafetyFilterNode()
  : Node("safety_filter")
  {
    enable_aeb_ = declare_parameter<bool>("enable_aeb", true);
    ttc_threshold_ = declare_parameter<double>("ttc_threshold", 0.4);
    half_width_ = declare_parameter<double>("vehicle_half_width", 0.15);
    lateral_margin_ = declare_parameter<double>("lateral_margin", 0.05);
    max_check_range_ = declare_parameter<double>("max_check_range", 5.0);
    const double release = declare_parameter<double>("release_time", 0.5);
    max_speed_ = declare_parameter<double>("max_speed", 3.0);
    max_steer_ = declare_parameter<double>("max_steering_angle", 0.4189);
    drive_timeout_ = declare_parameter<double>("drive_timeout", 0.2);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.2);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.2);
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    latch_ = race_control::AebLatch(release);

    sub_drive_ = create_subscription<AckermannDriveStamped>(
      "/drive_raw", 10, std::bind(&SafetyFilterNode::on_drive, this, std::placeholders::_1));
    sub_scan_ = create_subscription<LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetyFilterNode::on_scan, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        v_ = m->twist.twist.linear.x;
        odom_rx_ = now().seconds();
      });
    pub_drive_ = create_publisher<AckermannDriveStamped>("/drive", 10);
    pub_status_ = create_publisher<std_msgs::msg::String>("/safety/status", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&SafetyFilterNode::watchdog, this));
  }

private:
  void on_scan(const LaserScan::SharedPtr msg)
  {
    scan_rx_ = now().seconds();
    if (!enable_aeb_) {return;}
    std::vector<double> r(msg->ranges.begin(), msg->ranges.end());
    const double ttc = race_control::min_ttc_straight(
      r, msg->angle_min, msg->angle_increment, v_, half_width_ + lateral_margin_,
      max_check_range_);
    latch_.update(ttc < ttc_threshold_, scan_rx_);
  }

  // 현재 상태 판정. 빈 문자열이면 OK
  std::string fault(double t)
  {
    if (t - scan_rx_ > scan_timeout_) {return "SCAN_TIMEOUT";}
    if (t - odom_rx_ > odom_timeout_) {return "ODOM_TIMEOUT";}
    if (enable_aeb_ && latch_.active()) {return "AEB";}
    return "";
  }

  void out(double steer, double speed, const std::string & status)
  {
    AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = static_cast<float>(steer);
    cmd.drive.speed = static_cast<float>(speed);
    pub_drive_->publish(cmd);
    if (status != last_status_) {
      if (status != "OK") {
        RCLCPP_WARN(get_logger(), "safety: %s", status.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "safety: OK");
      }
      last_status_ = status;
    }
    std_msgs::msg::String s;
    s.data = status;
    pub_status_->publish(s);
  }

  void on_drive(const AckermannDriveStamped::SharedPtr msg)
  {
    const double t = now().seconds();
    drive_rx_ = t;
    const auto c = race_control::sanitize_command(
      msg->drive.steering_angle, msg->drive.speed, max_steer_, max_speed_);
    if (!c.ok) {out(last_steer_, 0.0, "INVALID_CMD"); return;}
    last_steer_ = c.steering;
    const std::string f = fault(t);
    if (!f.empty()) {out(c.steering, 0.0, f); return;}
    out(c.steering, c.speed, "OK");
  }

  void watchdog()
  {
    const double t = now().seconds();
    if (drive_rx_ > 0.0 && t - drive_rx_ > drive_timeout_) {
      out(last_steer_, 0.0, "CMD_TIMEOUT");
    }
  }

  bool enable_aeb_{};
  double ttc_threshold_{}, half_width_{}, lateral_margin_{}, max_check_range_{};
  double max_speed_{}, max_steer_{}, drive_timeout_{}, scan_timeout_{}, odom_timeout_{};
  race_control::AebLatch latch_;
  double v_{0.0}, drive_rx_{0.0}, scan_rx_{0.0}, odom_rx_{0.0}, last_steer_{0.0};
  std::string last_status_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr sub_drive_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
