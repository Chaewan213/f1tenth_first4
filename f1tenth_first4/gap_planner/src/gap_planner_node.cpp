// gap_planner_node.cpp
// /scan -> Follow the Gap -> /plan/gap (race_msgs/LocalPlan)
// 이 노드는 /drive를 직접 발행하지 않는다. 조향은 tracker 몫이다.
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "gap_planner/gap_core.hpp"
#include "race_msgs/msg/local_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using race_msgs::msg::LocalPlan;
using sensor_msgs::msg::LaserScan;

class GapPlannerNode : public rclcpp::Node
{
public:
  GapPlannerNode()
  : Node("gap_planner")
  {
    namespace gc = gap_planner;
    p_.field_of_view = gc::deg2rad(declare_parameter<double>("field_of_view_deg", 180.0));
    p_.max_range = declare_parameter<double>("max_range", 10.0);
    p_.smoothing_window = declare_parameter<int>("smoothing_window", 5);
    p_.bubble_radius = declare_parameter<double>("bubble_radius", 0.45);
    p_.front_half_angle = gc::deg2rad(declare_parameter<double>("front_half_angle_deg", 5.0));
    p_.straight_speed = declare_parameter<double>("straight_speed", 1.5);
    p_.corner_speed = declare_parameter<double>("corner_speed", 1.0);
    p_.sharp_corner_speed = declare_parameter<double>("sharp_corner_speed", 0.5);
    p_.front_slowdown_distance = declare_parameter<double>("front_slowdown_distance", 2.5);
    p_.front_stop_distance = declare_parameter<double>("front_stop_distance", 0.7);
    p_.corner_angle = gc::deg2rad(declare_parameter<double>("corner_angle_deg", 10.0));
    p_.sharp_corner_angle = gc::deg2rad(declare_parameter<double>("sharp_corner_angle_deg", 20.0));
    p_.side_window_min = gc::deg2rad(declare_parameter<double>("side_window_min_deg", 60.0));
    p_.side_window_max = gc::deg2rad(declare_parameter<double>("side_window_max_deg", 90.0));
    p_.corridor_side_max = declare_parameter<double>("corridor_side_max", 1.5);

    path_step_ = declare_parameter<double>("path_step", 0.25);
    path_max_len_ = declare_parameter<double>("path_max_len", 2.0);
    path_min_len_ = declare_parameter<double>("path_min_len", 0.5);
    heading_alpha_ = declare_parameter<double>("heading_filter_alpha", 0.7);
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");

    pub_ = create_publisher<LocalPlan>("/plan/gap", 10);
    sub_ = create_subscription<LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&GapPlannerNode::on_scan, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "gap_planner started (scan: %s)", scan_topic.c_str());
  }

private:
  void on_scan(const LaserScan::SharedPtr msg)
  {
    std::vector<double> ranges(msg->ranges.begin(), msg->ranges.end());
    auto r = gap_planner::compute(
      ranges, msg->angle_min, msg->angle_max, msg->angle_increment, p_);

    LocalPlan plan;
    plan.header.stamp = msg->header.stamp;
    plan.header.frame_id = "base_link";  // 확인필요: laser와 base_link 사이 오프셋은 현재 무시
    plan.source = LocalPlan::SOURCE_GAP;
    plan.free_distance = static_cast<float>(r.target_range);

    if (!r.valid) {
      // 주행 가능한 갭이 없음: 점 없이 신뢰도 0 + 속도 0 으로 알린다
      plan.confidence = 0.0f;
      plan.speed_limit = 0.0f;
      pub_->publish(plan);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "no drivable gap");
      return;
    }

    heading_f_ = has_prev_ ? gap_planner::filter_heading(heading_f_, r.heading, heading_alpha_)
                           : r.heading;
    has_prev_ = true;

    for (const auto & xy : gap_planner::heading_path(
        heading_f_, r.target_range, path_step_, path_max_len_, path_min_len_))
    {
      geometry_msgs::msg::Point pt;
      pt.x = xy.first;
      pt.y = xy.second;
      plan.points.push_back(pt);
    }
    plan.speed_limit = static_cast<float>(r.speed_limit);
    plan.confidence = static_cast<float>(r.corridor_confidence);
    pub_->publish(plan);
  }

  gap_planner::GapParams p_;
  double path_step_{}, path_max_len_{}, path_min_len_{}, heading_alpha_{};
  double heading_f_{0.0};
  bool has_prev_{false};
  rclcpp::Publisher<LocalPlan>::SharedPtr pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GapPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
