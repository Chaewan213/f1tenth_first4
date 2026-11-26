# 인터페이스 규약

이 문서와 `race_msgs`를 바꾸는 PR은 팀 전원 합의가 필요합니다.

## 좌표계와 단위

- 모든 경로점은 `base_link` 기준: x 전방(+), y 좌측(+), 단위 m
- 각도 rad, 좌회전(+). 속도 m/s
- `header.stamp`는 원본 센서(이미지·스캔)의 시각을 그대로 복사

## race_msgs/LocalPlan

| 필드 | 의미 |
|---|---|
| `points` | 경로점, 차량에서 가까운 순. 점이 1개면 tracker는 그 점을 그대로 추종 |
| `speed_limit` | 이 계획이 허용하는 최대 속도 |
| `free_distance` | 경로 방향 여유거리. 모르면 -1.0 |
| `confidence` | 0~1. lane = 차선 검출 품질, gap = 통로(좌우 벽) 검출 정도 |
| `source` | `SOURCE_LANE`(0) / `SOURCE_GAP`(1). tracker가 모드별 파라미터 선택에 사용 |

## 토픽

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/flir_camera/image_raw` | sensor_msgs/Image (bayer_rggb8) | 카메라 드라이버 (spinnaker) | lane_planner |
| `/scan` | sensor_msgs/LaserScan | 라이다 드라이버 | gap_planner, safety_filter |
| `/odom` | nav_msgs/Odometry | VESC | tracker, safety_filter |
| `/plan/lane` | race_msgs/LocalPlan | lane_planner | mode_manager |
| `/plan/gap` | race_msgs/LocalPlan | gap_planner | mode_manager |
| `/plan/active` | race_msgs/LocalPlan | mode_manager | tracker |
| `/mode` | std_msgs/String (`SAFE`/`LANE`/`GAP`) | mode_manager | tracker |
| `/drive_raw` | ackermann_msgs/AckermannDriveStamped | tracker | safety_filter |
| `/drive` | ackermann_msgs/AckermannDriveStamped | safety_filter | ackermann_mux |
| `/safety/status` | std_msgs/String | safety_filter | (로그·모니터링) |

## 각 노드가 하지 않는 일

- 플래너: 조향 계산, 모드 판단, `/drive` 발행
- mode_manager: 조향·속도 계산 (전환 직후 속도 상한만 낮춤)
- tracker: 모드 판단, 센서 직접 처리
- safety_filter: 회피 조향, 속도 계획. 평소에는 명령을 그대로 통과

## 모드 전환 규칙 (mode_manager)

- SAFE → LANE/GAP: 해당 계획의 신뢰도가 높은 상태가 `confirm_frames`틱 연속
- LANE → GAP: 차선 신뢰도 < `lane_conf_low` 이고 통로 신뢰도 ≥ `gap_conf_high` 가 연속
- GAP → LANE: 통로 신뢰도 < `gap_conf_low` 이고 차선 신뢰도 ≥ `lane_conf_high` 가 연속
- 현재 모드의 계획이 끊기고 다른 쪽도 쓸 수 없으면 SAFE (tracker 정지)
- 전환 후 `transition_hold_time` 동안 속도 상한 `transition_speed_limit`
