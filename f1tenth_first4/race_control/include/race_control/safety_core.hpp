// safety_core.hpp
// ROS 의존성 없는 안전 판정 로직: TTC 기반 AEB, 명령 유효성 검사.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace race_control
{

// 전방 차폭 통로(직진 가정) 안의 빔만으로 최소 TTC 계산.
// TTC_i = r_i / (v cos θ_i), v cos θ_i <= 0 인 빔은 접근하지 않으므로 제외.
// v1: 직진 통로. v2 계획: 현재 조향의 예측 원호를 따라 검사 (docs 참고).
inline double min_ttc_straight(
  const std::vector<double> & ranges, double angle_min, double angle_inc,
  double v, double half_width, double max_check_range)
{
  double best = std::numeric_limits<double>::infinity();
  if (v <= 1e-3 || angle_inc <= 0.0) {return best;}
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r <= 0.0 || r > max_check_range) {continue;}
    const double th = angle_min + static_cast<double>(i) * angle_inc;
    const double x = r * std::cos(th);
    const double y = r * std::sin(th);
    if (x <= 0.0 || std::abs(y) > half_width) {continue;}
    const double closing = v * std::cos(th);
    if (closing <= 1e-6) {continue;}
    best = std::min(best, r / closing);
  }
  return best;
}

struct CommandCheck
{
  bool ok = true;      // false면 NaN/Inf 등으로 명령 자체가 무효
  double steering = 0.0;
  double speed = 0.0;
};

inline CommandCheck sanitize_command(
  double steering, double speed, double max_steer, double max_speed)
{
  CommandCheck c;
  if (!std::isfinite(steering) || !std::isfinite(speed)) {
    c.ok = false;
    return c;
  }
  c.steering = std::clamp(steering, -max_steer, max_steer);
  c.speed = std::clamp(speed, -max_speed, max_speed);
  return c;
}

// AEB 래치: 발동 후 release_time 동안 TTC가 안전해야 해제 (3바퀴 완주를 위해 자동 해제)
class AebLatch
{
public:
  explicit AebLatch(double release_time = 0.5)
  : release_time_(release_time) {}

  bool update(bool danger, double now)
  {
    if (danger) {
      active_ = true;
      last_danger_ = now;
    } else if (active_ && now - last_danger_ >= release_time_) {
      active_ = false;
    }
    return active_;
  }
  bool active() const {return active_;}

private:
  double release_time_;
  bool active_{false};
  double last_danger_{0.0};
};

}  // namespace race_control
