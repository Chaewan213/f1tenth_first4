// mode_manager_node.cpp
// /plan/lane, /plan/gap -> 상태머신 -> /plan/active, /mode, /plan/active_path(RViz)
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "race_control/mode_core.hpp"
#include "race_msgs/msg/local_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using race_msgs::msg::LocalPlan;
using race_control::Mode;

class ModeManagerNode : public rclcpp::Node
{
public:
  ModeManagerNode()
  : Node("mode_manager"), sm_(race_control::ModeParams{})
  {
    race_control::ModeParams p;
    p.lane_conf_high = declare_parameter<double>("lane_conf_high", 0.6);
    p.lane_conf_low = declare_parameter<double>("lane_conf_low", 0.3);
    p.gap_conf_high = declare_parameter<double>("gap_conf_high", 0.75);
    p.gap_conf_low = declare_parameter<double>("gap_conf_low", 0.25);
    p.confirm_frames = declare_parameter<int>("confirm_frames", 5);
    const auto initial = declare_parameter<std::string>("initial_mode", "LANE");
    const auto force = declare_parameter<std::string>("force_mode", "AUTO");
    plan_timeout_ = declare_parameter<double>("plan_timeout", 0.3);
    transition_hold_ = declare_parameter<double>("transition_hold_time", 0.5);
    transition_speed_ = declare_parameter<double>("transition_speed_limit", 0.8);
    const double rate = declare_parameter<double>("rate_hz", 50.0);

    bool ok = true;
    p.initial_mode = race_control::parse_mode(initial, ok);
    if (!ok || p.initial_mode == Mode::SAFE) {
      RCLCPP_WARN(get_logger(), "initial_mode '%s' invalid, using LANE", initial.c_str());
      p.initial_mode = Mode::LANE;
    }
    if (!force.empty() && force != "AUTO") {
      p.force_mode = race_control::parse_mode(force, ok);
      p.force_enabled = ok && p.force_mode != Mode::SAFE;
      if (p.force_enabled) {
        RCLCPP_WARN(get_logger(), "force_mode=%s (테스트용 고정 모드)", force.c_str());
      }
    }
    sm_ = race_control::ModeStateMachine(p);

    sub_lane_ = create_subscription<LocalPlan>(
      "/plan/lane", 10, [this](LocalPlan::SharedPtr m) {lane_ = m; lane_rx_ = now_s();});
    sub_gap_ = create_subscription<LocalPlan>(
      "/plan/gap", 10, [this](LocalPlan::SharedPtr m) {gap_ = m; gap_rx_ = now_s();});
    pub_plan_ = create_publisher<LocalPlan>("/plan/active", 10);
    pub_mode_ = create_publisher<std_msgs::msg::String>("/mode", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/plan/active_path", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&ModeManagerNode::tick, this));
  }

private:
  double now_s() {return now().seconds();}

  void tick()
  {
    const double t = now_s();
    race_control::ModeInputs in;
    in.lane_fresh = lane_ && (t - lane_rx_) < plan_timeout_;
    in.gap_fresh = gap_ && (t - gap_rx_) < plan_timeout_;
    in.lane_conf = lane_ ? lane_->confidence : 0.0;
    in.gap_conf = gap_ ? gap_->confidence : 0.0;

    bool transitioned = false;
    const Mode m = sm_.step(in, transitioned);
    if (transitioned) {
      last_transition_ = t;
      RCLCPP_INFO(
        get_logger(), "mode -> %s (lane_conf=%.2f fresh=%d, gap_conf=%.2f fresh=%d, L->G=%d, G->L=%d)",
        race_control::mode_name(m), in.lane_conf, in.lane_fresh, in.gap_conf, in.gap_fresh,
        sm_.lane_to_gap_count(), sm_.gap_to_lane_count());
    }

    std_msgs::msg::String mode_msg;
    mode_msg.data = race_control::mode_name(m);
    pub_mode_->publish(mode_msg);

    if (m == Mode::SAFE) {return;}  // 계획을 내지 않음 -> tracker가 정지

    LocalPlan out = (m == Mode::LANE) ? *lane_ : *gap_;
    if (t - last_transition_ < transition_hold_) {
      out.speed_limit = std::min(out.speed_limit, static_cast<float>(transition_speed_));
    }
    pub_plan_->publish(out);

    nav_msgs::msg::Path path;
    path.header = out.header;
    for (const auto & pt : out.points) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position = pt;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);
  }

  race_control::ModeStateMachine sm_;
  LocalPlan::SharedPtr lane_, gap_;
  double lane_rx_{0.0}, gap_rx_{0.0}, last_transition_{-1e9};
  double plan_timeout_{}, transition_hold_{}, transition_speed_{};
  rclcpp::Subscription<LocalPlan>::SharedPtr sub_lane_, sub_gap_;
  rclcpp::Publisher<LocalPlan>::SharedPtr pub_plan_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
