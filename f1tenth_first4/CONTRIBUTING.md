# 팀 규칙

## 이름 규칙 (ROS 2 관례)

| 대상 | 규칙 | 예시 |
|---|---|---|
| 패키지 | 소문자 + 숫자 + `_` | `gap_planner` |
| 노드, 실행파일 | snake_case | `gap_planner_node` |
| C++/Python 파일 | snake_case | `gap_core.hpp`, `lane_core.py` |
| 클래스 | PascalCase | `GapPlannerNode` |
| 함수, 변수 | snake_case | `find_max_gap()` |
| 상수 | UPPER_CASE | `SOURCE_LANE` |
| msg 파일 | CamelCase | `LocalPlan.msg` |
| 토픽 | 소문자, `/`로 묶기 | `/plan/lane` |
| 파라미터 | snake_case, 점으로 그룹 | `lane.lookahead_min` |
| rosbag | `날짜_시간_내용` | `20261124_1910_lap3_full` |

## 브랜치·커밋

- `main`은 항상 실차에서 돌아가는 상태. 직접 push 금지, PR로만 merge
- 브랜치: `<종류>/<패키지>-<내용>` 예: `feat/gap-planner-hysteresis`
- 커밋: `<종류>(<패키지>): <요약>` 예: `tune(tracker): gap lookahead 0.4→0.5`
- 종류: `feat` `fix` `tune` `refactor` `docs` `test` `chore`

## PR

- PR 하나에 기능 하나
- `bash tools/run_core_tests.sh`와 `colcon build` 통과
- 테스트 방법과 파라미터 변경 내역을 템플릿에 기입
- 1명 이상 승인 후 merge
- `race_msgs`, 토픽 이름, 좌표계, 단위 변경은 전원 합의 + `docs/interfaces.md` 동시 수정

## 코드 구조 원칙

- 알고리즘은 ROS 없는 core 파일(`*_core.hpp`, `lane_core.py`)에 두고 단위 테스트를 붙인다
- 노드 파일은 토픽 연결과 파라미터 읽기만 한다
- 플래너는 `/drive`를 직접 발행하지 않는다

## 실차

- 차량에는 main 또는 태그 버전만 올린다
- 현장에서 바꾼 파라미터는 당일 `tune/` 브랜치로 커밋한다
- rosbag은 레포에 올리지 않고 공유 드라이브에 보관하며, 촬영 당시 커밋 해시를 메모한다
