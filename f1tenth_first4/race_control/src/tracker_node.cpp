// tracker_node.cpp
// /plan/active + /odom + /mode -> Pure Pursuit 조향 + 속도 계획 -> /drive_raw
// 모드별 파라미터 세트(lane.*, gap.*)를 /mode에 따라 바꿔 쓴다. 제어기는 하나.
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "race_control/tracker_core.hpp"
#include "race_msgs/msg/local_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using race_msgs::msg::LocalPlan;

struct ModeGains
{
  double lookahead_k, lookahead_min, lookahead_max, lookahead_l0, v_max;
};

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode()
  : Node("tracker")
  {
    algorithm_ = declare_parameter<std::string>("algorithm", "pure_pursuit");
    if (algorithm_ != "pure_pursuit") {
      RCLCPP_WARN(get_logger(), "algorithm '%s' 미구현, pure_pursuit 사용", algorithm_.c_str());
    }
    wheelbase_ = declare_parameter<double>("wheelbase", 0.33);
    max_steer_ = declare_parameter<double>("max_steering_angle", 0.4189);
    max_steer_rate_ = declare_parameter<double>("max_steering_rate", 3.0);
    a_lat_max_ = declare_parameter<double>("a_lat_max", 3.0);
    a_brake_ = declare_parameter<double>("a_brake", 3.0);
    max_accel_ = declare_parameter<double>("max_accel", 2.0);
    stop_margin_ = declare_parameter<double>("stop_margin", 0.3);
    plan_timeout_ = declare_parameter<double>("plan_timeout", 0.3);
    const double rate = declare_parameter<double>("rate_hz", 50.0);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    lane_ = load_gains("lane", 0.3, 0.6, 0.6, 2.0, 2.0);
    gap_ = load_gains("gap", 0.2, 0.4, 0.4, 1.5, 1.5);
    dt_ = 1.0 / rate;

    sub_plan_ = create_subscription<LocalPlan>(
      "/plan/active", 10, [this](LocalPlan::SharedPtr m) {plan_ = m; plan_rx_ = now().seconds();});
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {v_ = m->twist.twist.linear.x;});
    sub_mode_ = create_subscription<std_msgs::msg::String>(
      "/mode", 10, [this](std_msgs::msg::String::SharedPtr m) {mode_ = m->data;});
    pub_drive_ = create_publisher<AckermannDriveStamped>("/drive_raw", 10);
    pub_target_ = create_publisher<geometry_msgs::msg::PointStamped>("/tracker/target", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 * dt_)),
      std::bind(&TrackerNode::tick, this));
  }

private:
  ModeGains load_gains(
    const std::string & ns, double k, double l0, double lmin, double lmax, double vmax)
  {
    ModeGains g;
    g.lookahead_k = declare_parameter<double>(ns + ".lookahead_k", k);
    g.lookahead_l0 = declare_parameter<double>(ns + ".lookahead_l0", l0);
    g.lookahead_min = declare_parameter<double>(ns + ".lookahead_min", lmin);
    g.lookahead_max = declare_parameter<double>(ns + ".lookahead_max", lmax);
    g.v_max = declare_parameter<double>(ns + ".v_max", vmax);
    return g;
  }

  void publish(double steer, double speed)
  {
    AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = static_cast<float>(steer);
    cmd.drive.speed = static_cast<float>(speed);
    pub_drive_->publish(cmd);
  }

  void tick()
  {
    const bool fresh = plan_ && (now().seconds() - plan_rx_) < plan_timeout_;
    if (!fresh || mode_ == "SAFE" || plan_->points.empty()) {
      speed_cmd_ = 0.0;
      publish(steer_cmd_, 0.0);
      return;
    }
    const ModeGains & g = (plan_->source == LocalPlan::SOURCE_LANE) ? lane_ : gap_;

    std::vector<race_control::Point2> pts;
    pts.reserve(plan_->points.size());
    for (const auto & p : plan_->points) {pts.push_back({p.x, p.y});}

    const double ld = race_control::compute_lookahead(
      v_, g.lookahead_k, g.lookahead_l0, g.lookahead_min, g.lookahead_max);
    const auto r = race_control::pure_pursuit(pts, ld, wheelbase_, max_steer_);
    if (!r.valid) {
      speed_cmd_ = 0.0;
      publish(steer_cmd_, 0.0);
      return;
    }

    steer_cmd_ = race_control::rate_limit(steer_cmd_, r.steering, max_steer_rate_, dt_);
    const double v_target = race_control::compute_speed(
      r.curvature, plan_->free_distance, plan_->speed_limit, g.v_max,
      a_lat_max_, a_brake_, stop_margin_);
    speed_cmd_ = race_control::accel_limit(speed_cmd_, v_target, max_accel_, dt_);
    publish(steer_cmd_, speed_cmd_);

    geometry_msgs::msg::PointStamped tgt;
    tgt.header.stamp = plan_->header.stamp;
    tgt.header.frame_id = "base_link";
    tgt.point.x = r.target.x;
    tgt.point.y = r.target.y;
    pub_target_->publish(tgt);
  }

  std::string algorithm_, mode_{"SAFE"};
  double wheelbase_{}, max_steer_{}, max_steer_rate_{}, a_lat_max_{}, a_brake_{};
  double max_accel_{}, stop_margin_{}, plan_timeout_{}, dt_{};
  ModeGains lane_{}, gap_{};
  LocalPlan::SharedPtr plan_;
  double plan_rx_{0.0}, v_{0.0}, steer_cmd_{0.0}, speed_cmd_{0.0};
  rclcpp::Subscription<LocalPlan>::SharedPtr sub_plan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr pub_drive_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
