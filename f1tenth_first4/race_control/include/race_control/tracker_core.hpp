// tracker_core.hpp
// ROS 의존성 없는 경로 추종 로직. 현재 구현: Pure Pursuit (적응형 lookahead).
// 다른 알고리즘(Stanley, LQR, MPC 등)은 같은 입력(경로점)·출력(조향, 곡률)으로 추가한다.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace race_control
{

struct Point2
{
  double x;
  double y;
};

struct TrackResult
{
  bool valid = false;
  double steering = 0.0;   // [rad], 좌측 +
  double curvature = 0.0;  // [1/m]
  Point2 target{0.0, 0.0};
  double target_dist = 0.0;
};

inline double compute_lookahead(double v, double k, double l0, double lmin, double lmax)
{
  return std::clamp(k * std::max(v, 0.0) + l0, lmin, lmax);
}

// Pure Pursuit
// - 차량 원점에서 lookahead 이상 떨어진 첫 점을 목표로 삼는다.
// - 그런 점이 없으면 마지막 점을 쓴다 (경로점이 1개뿐인 학습 waypoint 방식도 그대로 동작).
inline TrackResult pure_pursuit(
  const std::vector<Point2> & pts, double lookahead, double wheelbase, double max_steer)
{
  TrackResult r;
  const Point2 * target = nullptr;
  for (const auto & p : pts) {
    const double d = std::hypot(p.x, p.y);
    if (d < 1e-3) {continue;}
    target = &p;
    if (d >= lookahead) {break;}
  }
  if (target == nullptr) {return r;}
  const double ld = std::hypot(target->x, target->y);
  const double alpha = std::atan2(target->y, target->x);
  r.curvature = 2.0 * std::sin(alpha) / ld;
  r.steering = std::clamp(std::atan(wheelbase * r.curvature), -max_steer, max_steer);
  r.target = *target;
  r.target_dist = ld;
  r.valid = true;
  return r;
}

// 속도 = min(모드 최고속, 횡가속 한계, 제동거리 한계, 계획의 speed_limit)
// free_distance < 0 이면 제동거리 항은 무시한다.
inline double compute_speed(
  double curvature, double free_distance, double plan_speed_limit,
  double v_max, double a_lat_max, double a_brake, double stop_margin)
{
  double v = std::min(v_max, plan_speed_limit);
  if (std::abs(curvature) > 1e-6) {
    v = std::min(v, std::sqrt(a_lat_max / std::abs(curvature)));
  }
  if (free_distance >= 0.0) {
    const double d = std::max(free_distance - stop_margin, 0.0);
    v = std::min(v, std::sqrt(2.0 * a_brake * d));
  }
  return std::max(v, 0.0);
}

inline double rate_limit(double prev, double target, double max_rate, double dt)
{
  const double step = max_rate * dt;
  return prev + std::clamp(target - prev, -step, step);
}

// 가속만 제한하고 감속은 즉시 허용
inline double accel_limit(double prev, double target, double max_accel, double dt)
{
  if (target <= prev) {return target;}
  return std::min(target, prev + max_accel * dt);
}

}  // namespace race_control
