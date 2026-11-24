// gap_core.hpp
// ROS 의존성이 없는 Follow the Gap 핵심 로직.
// 기반 코드(reactive_node.py)의 전처리·Bubble·최대 Gap·최원점 선택·속도 결정을
// 동일한 규칙으로 옮겼다. 차이점은 조향각 대신 "목표 방향(heading)"을 돌려준다는 것.
// 조향각 계산은 race_control/tracker가 담당한다.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace gap_planner
{

inline double deg2rad(double d) {return d * M_PI / 180.0;}

struct GapParams
{
  double field_of_view = deg2rad(180.0);   // 탐색에 쓰는 전방 시야 [rad]
  double max_range = 10.0;                 // 거리 상한 [m]
  int smoothing_window = 5;                // 이동평균 빔 개수
  double bubble_radius = 0.45;             // 최근접 장애물 주변 안전 반경 [m]
  double front_half_angle = deg2rad(5.0);  // 전방 대표거리 계산 범위 ±[rad]

  // 속도 상한 (기반 코드 determine_speed와 동일 규칙)
  double straight_speed = 1.5;
  double corner_speed = 1.0;
  double sharp_corner_speed = 0.5;
  double front_slowdown_distance = 2.5;
  double front_stop_distance = 0.7;
  double corner_angle = deg2rad(10.0);
  double sharp_corner_angle = deg2rad(20.0);

  // 통로(파이프 벽) 검출: 좌우 측면 창 안에 벽이 있으면 통로로 판단
  double side_window_min = deg2rad(60.0);
  double side_window_max = deg2rad(90.0);
  double corridor_side_max = 1.5;          // 이 거리 안에 벽이 있으면 "벽 있음" [m] (확인필요)
};

struct GapResult
{
  bool valid = false;
  double heading = 0.0;              // 목표 방향 [rad], 좌측 +
  double target_range = 0.0;         // 목표 빔의 거리 [m]
  double front_distance = 0.0;       // 정면 ±front_half_angle 중앙값 [m]
  double speed_limit = 0.0;          // [m/s]
  double corridor_confidence = 0.0;  // 0, 0.5, 1.0
  int gap_start = 0;
  int gap_end = 0;
  int best_index = 0;                // 원본 scan 기준 인덱스
};

// NaN·inf 처리, [0, max_range] 제한, 'same' 이동평균 (numpy.convolve(mode='same')와 동일)
inline std::vector<double> preprocess(const std::vector<double> & ranges, const GapParams & p)
{
  std::vector<double> out(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    double r = ranges[i];
    if (std::isnan(r)) {r = 0.0;} else if (std::isinf(r)) {r = (r > 0) ? p.max_range : 0.0;}
    out[i] = std::clamp(r, 0.0, p.max_range);
  }
  const int w = p.smoothing_window;
  if (w > 1 && static_cast<int>(out.size()) >= w) {
    const int n = static_cast<int>(out.size());
    const int offset = (w - 1) / 2;
    std::vector<double> sm(out.size(), 0.0);
    for (int i = 0; i < n; ++i) {
      double s = 0.0;
      for (int k = 0; k < w; ++k) {
        const int j = i + offset - k;
        if (j >= 0 && j < n) {s += out[j];}
      }
      sm[i] = s / w;
    }
    out.swap(sm);
  }
  return out;
}

// 0보다 큰 연속 구간 중 가장 긴 것. [start, end) 반환, 없으면 (0, 0). 동률이면 앞쪽.
inline std::pair<int, int> find_max_gap(const std::vector<double> & free_space)
{
  int best_s = 0, best_e = 0, cur_s = -1;
  const int n = static_cast<int>(free_space.size());
  for (int i = 0; i <= n; ++i) {
    const bool free_i = (i < n) && free_space[i] > 0.0;
    if (free_i && cur_s < 0) {cur_s = i;}
    if (!free_i && cur_s >= 0) {
      if (i - cur_s > best_e - best_s) {best_s = cur_s; best_e = i;}
      cur_s = -1;
    }
  }
  return {best_s, best_e};
}

// Gap 안에서 가장 먼 빔 (동률이면 앞쪽)
// 1e-9 허용오차: 이동평균 덧셈 순서 차이로 생기는 부동소수점 동률을 numpy.argmax와 같게 처리
inline int find_best_point(int s, int e, const std::vector<double> & ranges)
{
  if (e <= s) {return s;}
  int best = s;
  for (int i = s; i < e; ++i) {
    if (ranges[i] > ranges[best] + 1e-9) {best = i;}
  }
  return best;
}

inline double median_positive(const std::vector<double> & v)
{
  std::vector<double> pos;
  for (double x : v) {if (x > 0.0) {pos.push_back(x);}}
  if (pos.empty()) {return 0.0;}
  std::sort(pos.begin(), pos.end());
  const std::size_t m = pos.size() / 2;
  return (pos.size() % 2) ? pos[m] : 0.5 * (pos[m - 1] + pos[m]);
}

inline double determine_speed(double front_distance, double heading, const GapParams & p)
{
  const double a = std::abs(heading);
  if (front_distance < p.front_stop_distance) {return p.sharp_corner_speed;}
  if (front_distance < p.front_slowdown_distance || a > p.sharp_corner_angle) {
    return p.sharp_corner_speed;
  }
  if (a > p.corner_angle) {return p.corner_speed;}
  return p.straight_speed;
}

// 각도 [a0, a1] 에 해당하는 scan 인덱스 [s, e)
inline std::pair<int, int> angle_slice(
  double a0, double a1, double angle_min, double angle_inc, int n)
{
  int s = static_cast<int>(std::ceil((a0 - angle_min) / angle_inc));
  int e = static_cast<int>(std::floor((a1 - angle_min) / angle_inc)) + 1;
  s = std::clamp(s, 0, n);
  e = std::clamp(e, s, n);
  return {s, e};
}

inline double side_min_range(
  const std::vector<double> & scan, double a0, double a1,
  double angle_min, double angle_inc, const GapParams & p)
{
  auto [s, e] = angle_slice(a0, a1, angle_min, angle_inc, static_cast<int>(scan.size()));
  double m = std::numeric_limits<double>::infinity();
  for (int i = s; i < e; ++i) {
    const double r = scan[i];
    if (std::isfinite(r) && r > 0.0 && r <= p.max_range) {m = std::min(m, r);}
  }
  return m;
}

inline GapResult compute(
  const std::vector<double> & scan, double angle_min, double angle_max,
  double angle_inc, const GapParams & p)
{
  GapResult res;
  const int n = static_cast<int>(scan.size());
  if (n == 0 || angle_inc <= 0.0) {return res;}

  // 1) 전방 FOV 선택
  const double half = p.field_of_view / 2.0;
  auto [si, ei] = angle_slice(
    std::max(angle_min, -half), std::min(angle_max, half), angle_min, angle_inc, n);
  std::vector<double> front(scan.begin() + si, scan.begin() + ei);
  std::vector<double> proc = preprocess(front, p);
  if (proc.empty()) {return res;}

  // 2) 최근접 장애물
  int closest = -1;
  double closest_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(proc.size()); ++i) {
    if (proc[i] > 0.0 && proc[i] < closest_d) {closest_d = proc[i]; closest = i;}
  }

  // 3) Bubble
  std::vector<double> free_space = proc;
  if (closest >= 0) {
    const double bubble_angle = std::atan2(p.bubble_radius, std::max(closest_d, 1e-3));
    const int r = static_cast<int>(std::ceil(bubble_angle / angle_inc));
    const int bs = std::max(0, closest - r);
    const int be = std::min(static_cast<int>(free_space.size()), closest + r + 1);
    for (int i = bs; i < be; ++i) {free_space[i] = 0.0;}
  }

  // 4) 최대 Gap과 목표점
  auto [gs, ge] = find_max_gap(free_space);
  if (ge <= gs) {return res;}  // 주행 가능한 공간 없음
  const int best = find_best_point(gs, ge, free_space);

  res.gap_start = si + gs;
  res.gap_end = si + ge;
  res.best_index = si + best;
  res.heading = angle_min + res.best_index * angle_inc;
  res.target_range = free_space[best];

  // 5) 정면 대표거리와 속도 상한
  auto [fs, fe] = angle_slice(-p.front_half_angle, p.front_half_angle, angle_min, angle_inc, n);
  std::vector<double> fr(scan.begin() + fs, scan.begin() + fe);
  res.front_distance = median_positive(preprocess(fr, p));
  res.speed_limit = determine_speed(res.front_distance, res.heading, p);

  // 6) 통로 검출 신뢰도 (좌우 각각 벽이 있으면 0.5씩)
  const double left = side_min_range(
    scan, p.side_window_min, p.side_window_max, angle_min, angle_inc, p);
  const double right = side_min_range(
    scan, -p.side_window_max, -p.side_window_min, angle_min, angle_inc, p);
  res.corridor_confidence =
    0.5 * (left < p.corridor_side_max ? 1.0 : 0.0) + 0.5 * (right < p.corridor_side_max ? 1.0 : 0.0);

  res.valid = true;
  return res;
}

// 목표 방향으로 일정 간격 경로점 생성 (차량 기준 x 전방, y 좌측)
inline std::vector<std::pair<double, double>> heading_path(
  double heading, double target_range, double step, double max_len, double min_len)
{
  const double len = std::max(min_len, std::min(max_len, target_range));
  std::vector<std::pair<double, double>> pts;
  for (double s = step; s < len + 1e-9; s += step) {
    pts.emplace_back(s * std::cos(heading), s * std::sin(heading));
  }
  if (pts.empty()) {pts.emplace_back(len * std::cos(heading), len * std::sin(heading));}
  return pts;
}

// 각도 저역통과 (wrap 안전)
inline double filter_heading(double prev, double cur, double alpha)
{
  const double d = std::atan2(std::sin(cur - prev), std::cos(cur - prev));
  return prev + alpha * d;
}

}  // namespace gap_planner
