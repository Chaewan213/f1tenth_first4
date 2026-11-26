# f1tenth_first4

F1TENTH 레이스 주행 스택 (ROS 2 Humble, Jetson Orin Nano). 코스: 1/4 카메라 차선 구간 + 3/4 라이다 파이프 통로·장애물 구간, 3바퀴.

## 구조

```
카메라 ─▶ lane_planner ─▶ /plan/lane ┐
                                      ├▶ mode_manager ─▶ /plan/active ─▶ tracker ─▶ /drive_raw ─▶ safety_filter ─▶ /drive ─▶ ackermann_mux ─▶ VESC
라이다 ─▶ gap_planner  ─▶ /plan/gap  ┘        └▶ /mode                                          ▲ /scan, /odom
```

| 패키지 | 언어 | 노드 | 역할 |
|---|---|---|---|
| `race_msgs` | msg | - | 공통 인터페이스 `LocalPlan` |
| `gap_planner` | C++ | `gap_planner_node` | 라이다 Follow the Gap → 경로 |
| `lane_planner` | Python | `lane_planner_node` | 카메라 차선 → 경로 (classic / onnx 백엔드) |
| `race_control` | C++ | `mode_manager_node`, `tracker_node`, `safety_filter_node` | 모드 전환, Pure Pursuit, AEB·워치독 |
| `race_bringup` | launch | - | `race.launch.py`, `config/race_params.yaml` |

플래너는 `/drive`를 직접 내지 않습니다. 조향은 tracker 하나만 계산합니다. 세부 규약은 [docs/interfaces.md](docs/interfaces.md)를 보세요.

## 빌드

```bash
mkdir -p ~/race_ws/src && cd ~/race_ws/src
git clone https://github.com/Chaewan213/f1tenth_first4.git
cd ~/race_ws
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

## 실행

> 실차에서 전원을 켜고 SSH로 접속해 주행하는 전체 절차는 [docs/RUN_ON_CAR.md](docs/RUN_ON_CAR.md)를 따르세요. 요약: `bash tools/start_car.sh`

```bash
# 실차 (f1tenth_stack bringup을 먼저 실행, RB 데드맨을 누르고 있어야 /drive가 통과)
ros2 launch race_bringup race.launch.py

# 시뮬레이터(f1tenth_gym_ros)에서 라이다 구간만 테스트 (시뮬에는 카메라가 없음)
ros2 launch race_bringup race.launch.py use_lane:=false force_mode:=GAP odom_topic:=/ego_racecar/odom
```

확인용 토픽: `/mode`, `/safety/status`, `/plan/active_path`(RViz Path), `/tracker/target`(RViz Point), `/lane/debug_bev`(publish_debug_image: true일 때)

## 안 움직일 때 확인 순서

1. `ros2 topic echo /safety/status`: `ODOM_TIMEOUT`이면 `odom_topic`, `SCAN_TIMEOUT`이면 `scan_topic`이 틀린 것
2. `ros2 topic echo /mode`: 계속 `SAFE`면 플래너 신뢰도 확인 (`ros2 topic echo /plan/gap --field confidence`)
3. `/drive`는 나오는데 차가 안 움직이면 RB 데드맨, ackermann_mux 설정 확인

## 테스트 (ROS 없이)

```bash
bash tools/run_core_tests.sh
```

알고리즘 핵심부는 `*_core.hpp`, `lane_core.py`로 분리해 ROS 없이 테스트합니다. `gap_core`는 기반 코드 `reactive_node.py`와 300개 랜덤 스캔에서 조향·속도가 일치하는 것을 확인했습니다.

## 실차 전에 채울 값

[docs/TODO_VALUES.md](docs/TODO_VALUES.md)의 "확인필요" 항목을 먼저 채우세요. 특히 `homography`가 비어 있으면 lane_planner는 신뢰도 0만 내보내므로 LANE 모드로 들어가지 않습니다.

## 문서

- [docs/RUN_ON_CAR.md](docs/RUN_ON_CAR.md): 실차 전원 켜기 → SSH → 주행 → 종료 절차
- [docs/interfaces.md](docs/interfaces.md): 토픽·좌표계·단위 규약
- [docs/TODO_VALUES.md](docs/TODO_VALUES.md): 확인필요 값 목록
- [docs/camera_pipeline.md](docs/camera_pipeline.md): 카메라 캘리브레이션, Colab 학습, 모델 배포 절차
- [CONTRIBUTING.md](CONTRIBUTING.md): 이름·브랜치·커밋·PR 규칙
