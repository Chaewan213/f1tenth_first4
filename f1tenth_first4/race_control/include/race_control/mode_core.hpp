// mode_core.hpp
// ROS 의존성 없는 모드 상태머신: SAFE / LANE / GAP
// 3바퀴 반복을 위해 LANE <-> GAP 쌍방향 전이를 지원한다.
#pragma once

#include <string>

namespace race_control
{

enum class Mode { SAFE, LANE, GAP };

inline const char * mode_name(Mode m)
{
  switch (m) {
    case Mode::LANE: return "LANE";
    case Mode::GAP: return "GAP";
    default: return "SAFE";
  }
}

struct ModeParams
{
  double lane_conf_high = 0.6;   // 이 이상이면 차선 "좋음"
  double lane_conf_low = 0.3;    // 이 미만이면 차선 "나쁨"
  double gap_conf_high = 0.75;   // 이 이상이면 통로 "검출" (좌우 벽 모두 = 1.0)
  double gap_conf_low = 0.25;    // 이 미만이면 통로 "없음"
  int confirm_frames = 5;        // 조건이 연속 N틱 유지돼야 전환
  Mode initial_mode = Mode::LANE;
  Mode force_mode = Mode::SAFE;  // SAFE가 아니면 해당 모드로 고정 (시뮬·단독 테스트용)
  bool force_enabled = false;
};

struct ModeInputs
{
  bool lane_fresh = false;
  bool gap_fresh = false;
  double lane_conf = 0.0;
  double gap_conf = 0.0;
};

class ModeStateMachine
{
public:
  explicit ModeStateMachine(const ModeParams & p)
  : p_(p) {}

  // 한 틱 진행. 전이가 일어나면 transitioned=true
  Mode step(const ModeInputs & in, bool & transitioned)
  {
    const Mode before = mode_;
    const bool lane_ok = in.lane_fresh && in.lane_conf >= p_.lane_conf_high;
    const bool lane_bad = !in.lane_fresh || in.lane_conf < p_.lane_conf_low;
    const bool gap_ok = in.gap_fresh && in.gap_conf >= p_.gap_conf_high;
    const bool gap_bad = !in.gap_fresh || in.gap_conf < p_.gap_conf_low;

    if (p_.force_enabled) {
      const bool fresh = (p_.force_mode == Mode::LANE) ? in.lane_fresh : in.gap_fresh;
      mode_ = fresh ? p_.force_mode : Mode::SAFE;
    } else {
      switch (mode_) {
        case Mode::SAFE: {
          Mode cand = Mode::SAFE;
          if (lane_ok && gap_ok) {cand = p_.initial_mode;}
          else if (lane_ok) {cand = Mode::LANE;}
          else if (gap_ok) {cand = Mode::GAP;}
          counter_ = (cand != Mode::SAFE) ? counter_ + 1 : 0;
          if (counter_ >= p_.confirm_frames) {mode_ = cand;}
          break;
        }
        case Mode::LANE:
          if (!in.lane_fresh && !gap_ok) {mode_ = Mode::SAFE; break;}
          counter_ = (lane_bad && gap_ok) ? counter_ + 1 : 0;
          if (counter_ >= p_.confirm_frames) {mode_ = Mode::GAP; ++lane_to_gap_;}
          break;
        case Mode::GAP:
          if (!in.gap_fresh && !lane_ok) {mode_ = Mode::SAFE; break;}
          counter_ = (gap_bad && lane_ok) ? counter_ + 1 : 0;
          if (counter_ >= p_.confirm_frames) {mode_ = Mode::LANE; ++gap_to_lane_;}
          break;
      }
    }
    transitioned = (mode_ != before);
    if (transitioned) {counter_ = 0;}
    return mode_;
  }

  Mode mode() const {return mode_;}
  int lane_to_gap_count() const {return lane_to_gap_;}
  int gap_to_lane_count() const {return gap_to_lane_;}

private:
  ModeParams p_;
  Mode mode_{Mode::SAFE};
  int counter_{0};
  int lane_to_gap_{0};
  int gap_to_lane_{0};
};

inline Mode parse_mode(const std::string & s, bool & ok)
{
  ok = true;
  if (s == "LANE") {return Mode::LANE;}
  if (s == "GAP") {return Mode::GAP;}
  if (s == "SAFE") {return Mode::SAFE;}
  ok = false;
  return Mode::SAFE;
}

}  // namespace race_control
