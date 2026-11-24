// ROS 없이 g++만으로 빌드되는 tracker/safety/mode 코어 단위 테스트
#include <cmath>
#include <cstdio>
#include <vector>

#include "race_control/mode_core.hpp"
#include "race_control/safety_core.hpp"
#include "race_control/tracker_core.hpp"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

using namespace race_control;

int main()
{
  // ---------- tracker ----------
  {
    // 직선 경로 -> 조향 0
    auto r = pure_pursuit({{0.5, 0}, {1.0, 0}, {1.5, 0}}, 1.0, 0.33, 0.42);
    CHECK(r.valid && std::abs(r.steering) < 1e-9 && std::abs(r.target.x - 1.0) < 1e-9);
    // 왼쪽 목표 -> 좌조향(+), 곡률 = 2 sin(a)/ld
    auto l = pure_pursuit({{1.0, 1.0}}, 1.0, 0.33, 1.0);
    const double ld = std::sqrt(2.0);
    CHECK(l.valid && l.steering > 0 && std::abs(l.curvature - 2 * std::sin(M_PI / 4) / ld) < 1e-9);
    // 점 1개(학습 waypoint)면 lookahead와 무관하게 그 점
    auto w = pure_pursuit({{0.8, -0.2}}, 2.0, 0.33, 0.42);
    CHECK(w.valid && std::abs(w.target.x - 0.8) < 1e-9 && w.steering < 0);
    // 조향 포화
    auto s = pure_pursuit({{0.1, 1.0}}, 0.5, 0.33, 0.42);
    CHECK(std::abs(s.steering - 0.42) < 1e-9);
    // 빈 경로
    CHECK(!pure_pursuit({}, 1.0, 0.33, 0.42).valid);
    // lookahead clamp
    CHECK(std::abs(compute_lookahead(10.0, 0.3, 0.6, 0.6, 2.0) - 2.0) < 1e-9);
    // 속도: 곡률·여유거리·상한 중 최소, free_distance<0 이면 무시
    CHECK(std::abs(compute_speed(0.0, -1.0, 1.5, 2.0, 3.0, 3.0, 0.3) - 1.5) < 1e-9);
    CHECK(std::abs(compute_speed(3.0, -1.0, 5.0, 5.0, 3.0, 3.0, 0.3) - 1.0) < 1e-9);
    CHECK(compute_speed(0.0, 0.2, 5.0, 5.0, 3.0, 3.0, 0.3) == 0.0);
    // rate / accel limit
    CHECK(std::abs(rate_limit(0.0, 1.0, 3.0, 0.02) - 0.06) < 1e-9);
    CHECK(std::abs(accel_limit(1.0, 2.0, 2.0, 0.02) - 1.04) < 1e-9);
    CHECK(accel_limit(2.0, 0.0, 2.0, 0.02) == 0.0);
  }
  // ---------- safety ----------
  {
    // 정면 1m 장애물, 2m/s -> TTC 0.5s
    std::vector<double> r(181, 10.0);  // -90~+90도, 1도 간격
    r[90] = 1.0;
    const double inc = M_PI / 180.0;
    double ttc = min_ttc_straight(r, -M_PI / 2, inc, 2.0, 0.2, 5.0);
    CHECK(std::abs(ttc - 0.5) < 1e-9);
    // 차폭 통로 밖(옆 벽)은 무시
    std::vector<double> side(181, 10.0);
    side[0] = 0.3; side[180] = 0.3;
    CHECK(std::isinf(min_ttc_straight(side, -M_PI / 2, inc, 2.0, 0.2, 5.0)));
    // 정지 상태면 TTC 무한
    CHECK(std::isinf(min_ttc_straight(r, -M_PI / 2, inc, 0.0, 0.2, 5.0)));
    // 명령 검사
    CHECK(!sanitize_command(NAN, 1.0, 0.4, 3.0).ok);
    auto c = sanitize_command(1.0, 9.0, 0.4, 3.0);
    CHECK(c.ok && c.steering == 0.4 && c.speed == 3.0);
    // AEB 래치: 발동 후 release_time 동안 안전해야 해제
    AebLatch latch(0.5);
    CHECK(latch.update(true, 0.0));
    CHECK(latch.update(false, 0.3));
    CHECK(!latch.update(false, 0.6));
  }
  // ---------- mode ----------
  {
    ModeParams p;
    p.confirm_frames = 3;
    ModeStateMachine sm(p);
    bool tr = false;
    ModeInputs lane_sec{true, true, 0.9, 0.0};   // 차선 구간
    ModeInputs gap_sec{true, true, 0.1, 1.0};    // 통로 구간
    // 시작: SAFE -> LANE (3틱 확인)
    sm.step(lane_sec, tr); sm.step(lane_sec, tr);
    CHECK(sm.mode() == Mode::SAFE);
    sm.step(lane_sec, tr);
    CHECK(sm.mode() == Mode::LANE && tr);
    // 3바퀴: LANE->GAP->LANE 반복
    for (int lap = 0; lap < 3; ++lap) {
      for (int i = 0; i < 3; ++i) {sm.step(gap_sec, tr);}
      CHECK(sm.mode() == Mode::GAP);
      if (lap < 2) {
        for (int i = 0; i < 3; ++i) {sm.step(lane_sec, tr);}
        CHECK(sm.mode() == Mode::LANE);
      }
    }
    CHECK(sm.lane_to_gap_count() == 3 && sm.gap_to_lane_count() == 2);
    // 경계에서 흔들림(1틱씩 번갈아)이면 전환 안 됨
    ModeInputs mixed{true, true, 0.5, 0.5};
    for (int i = 0; i < 10; ++i) {sm.step((i % 2) ? lane_sec : mixed, tr);}
    CHECK(sm.mode() == Mode::GAP);
    // 두 계획 모두 끊기면 SAFE
    sm.step(ModeInputs{false, false, 0.0, 0.0}, tr);
    CHECK(sm.mode() == Mode::SAFE);
    // force_mode
    ModeParams f; f.force_enabled = true; f.force_mode = Mode::GAP;
    ModeStateMachine fs(f);
    fs.step(ModeInputs{false, true, 0.0, 0.0}, tr);
    CHECK(fs.mode() == Mode::GAP);
  }

  std::printf(failures ? "control_core: %d FAIL\n" : "control_core: all passed\n", failures);
  return failures ? 1 : 0;
}
