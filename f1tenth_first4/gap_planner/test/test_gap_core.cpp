// ROS 없이 g++만으로 빌드되는 gap_core 단위 테스트
#include <cmath>
#include <cstdio>
#include <vector>

#include "gap_planner/gap_core.hpp"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

using namespace gap_planner;

int main()
{
  GapParams p;

  // preprocess: NaN->0, +inf->max, 음수->0, 이동평균(numpy same과 동일)
  {
    GapParams q; q.smoothing_window = 1;
    auto o = preprocess({NAN, INFINITY, -1.0, 3.0}, q);
    CHECK(o[0] == 0.0 && o[1] == 10.0 && o[2] == 0.0 && o[3] == 3.0);
    auto s = preprocess({5, 5, 5, 5, 5}, p);  // [3,4,5,4,3]
    CHECK(std::abs(s[0] - 3.0) < 1e-9 && std::abs(s[2] - 5.0) < 1e-9 && std::abs(s[4] - 3.0) < 1e-9);
  }
  // find_max_gap: [start, end), 동률이면 앞쪽
  {
    auto g = find_max_gap({0, 1, 1, 0, 1, 1, 1, 0});
    CHECK(g.first == 4 && g.second == 7);
    auto t = find_max_gap({1, 1, 0, 1, 1});
    CHECK(t.first == 0 && t.second == 2);
    auto z = find_max_gap({0, 0});
    CHECK(z.first == 0 && z.second == 0);
  }
  // 좁은 통로(좌우 벽 0.8m) + 정면 장애물이 오른쪽에 있을 때 왼쪽으로 향해야 함
  {
    const int n = 1081;
    const double amin = -3 * M_PI / 4, amax = 3 * M_PI / 4, inc = (amax - amin) / (n - 1);
    std::vector<double> scan(n);
    for (int i = 0; i < n; ++i) {
      const double th = amin + i * inc;
      double r = 10.0;
      if (std::abs(std::sin(th)) > 1e-6) {r = std::min(r, 0.8 / std::abs(std::sin(th)));}
      // 정면 우측 1.5m 지점에 장애물(폭 0.3m, y=-0.3~0)
      const double x = 1.5;
      if (std::cos(th) > 0) {
        const double y = x * std::tan(th);
        if (y > -0.3 && y < 0.0) {r = std::min(r, x / std::cos(th));}
      }
      scan[i] = r;
    }
    auto r = compute(scan, amin, amax, inc, p);
    CHECK(r.valid);
    CHECK(r.heading > 0.0);                    // 왼쪽(+)으로 피함
    CHECK(r.corridor_confidence == 1.0);       // 좌우 벽 모두 검출
    auto path = heading_path(r.heading, r.target_range, 0.25, 2.0, 0.5);
    CHECK(!path.empty() && path.back().second > 0.0);
  }
  // 벽이 없는 열린 공간: 통로 신뢰도 0
  {
    std::vector<double> scan(1081, 10.0);
    auto r = compute(scan, -3 * M_PI / 4, 3 * M_PI / 4, (1.5 * M_PI) / 1080, p);
    CHECK(r.valid && r.corridor_confidence == 0.0);
  }
  // 모든 빔이 0(무효): 갭 없음
  {
    std::vector<double> scan(100, 0.0);
    auto r = compute(scan, -M_PI / 2, M_PI / 2, M_PI / 99, p);
    CHECK(!r.valid);
  }
  // heading 필터: wrap 안전
  CHECK(std::abs(filter_heading(0.0, 1.0, 0.5) - 0.5) < 1e-9);

  std::printf(failures ? "gap_core: %d FAIL\n" : "gap_core: all passed\n", failures);
  return failures ? 1 : 0;
}
