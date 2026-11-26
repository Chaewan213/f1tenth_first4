# 확인필요 값 목록

모든 값은 `race_bringup/config/race_params.yaml`에 있습니다. 실수 파라미터는 소수점을 붙여 쓰세요 (예: `1.0`).

## 1. 차량 하드웨어 (실측)

| 파라미터 | 노드 | 현재 값 | 확인 방법 |
|---|---|---|---|
| `wheelbase` | tracker | 0.33 | 앞바퀴 축 중심 ~ 뒷바퀴 축 중심 거리 실측 |
| `max_steering_angle` | tracker, safety_filter | 0.4189 (24°, 기반 코드 값) | 서보 최대 조향 시 바퀴 각도 측정. 두 노드 같은 값 |
| `vehicle_half_width` | safety_filter | 0.15 | 차폭(가장 넓은 곳) ÷ 2 |
| `a_brake` | tracker | 3.0 | 일정 속도에서 정지 명령 후 제동거리로 역산: a = v²/(2d) |
| `a_lat_max` | tracker | 3.0 | 원 선회 시험에서 미끄러지기 직전 v²/R |
| laser ↔ base_link 오프셋 | gap_planner | 무시 중 | 라이다가 뒷바퀴 축 중심에서 얼마나 앞에 있는지 실측 (필요하면 경로점에 더함) |

## 2. 센서·토픽

| 항목 | 현재 값 | 확인 방법 |
|---|---|---|
| 카메라 이미지 토픽 `image_topic` | /flir_camera/image_raw (수업 기준) | `ros2 topic list`로 확인 |
| 오도메트리 토픽 `odom_topic` | /odom | 실차 VESC 토픽 확인. 시뮬은 /ego_racecar/odom |
| 라이다 토픽 `scan_topic` | /scan | 실차 라이다 드라이버 확인 |

## 3. 카메라·차선 (docs/camera_pipeline.md 순서대로)

| 파라미터 | 현재 값 | 확인 방법 |
|---|---|---|
| `homography` (9개) | 전부 0 (미설정) | `calibrate_homography` 실행 결과 붙여 넣기 |
| `m_per_px`, `origin_u`, `origin_v` | 0.005, 200.0, 440.0 | BEV 해상도와 차량 원점 위치 결정 (캘리브레이션과 같은 값 사용) |
| `camera_matrix`, `dist_coeffs` | 전부 0 (보정 안 함) | 우리 카메라의 `ost.yaml` 값을 옮김 |
| `hsv_low`, `hsv_high` | (18,50,120)~(36,255,255) 수업 기준값 | 조명에 맞춰 조정 (`publish_debug_image: true`로 확인) |
| `lane_width_m` | 0.80 (수업 예시) | 우리 트랙 차선 폭 실측 (두 테이프 중심 간격) |

## 4. 학습 모델 (backend: onnx 사용 시)

| 파라미터 | 확인할 것 |
|---|---|
| `model_path` | Jetson에 복사한 .onnx 경로 |
| `model_input_width/height` | 학습 코드의 입력 크기 |
| `model_mean/std`, `model_rgb` | 학습 코드의 정규화와 색 순서 |
| `model_y_sign` | 수업 라벨은 Y 좌측(+) → 1.0 |
| `model_origin_offset_x` | 뒷바퀴 축 중심에서 카메라 렌즈 바로 아래 바닥점까지 앞쪽 거리 [m] 실측 |

## 5. 코스 (트랙에서 조정)

| 파라미터 | 노드 | 현재 값 | 확인 방법 |
|---|---|---|---|
| `corridor_side_max` | gap_planner | 1.5 | 파이프 통로 폭의 절반 + 여유. 차선 구간에서는 벽이 이 거리 밖이어야 함 |
| `ttc_threshold` | safety_filter | 0.4 | 커브 입구에서 오작동하지 않는 최소값을 저속부터 찾기 |
| `lane_conf_*`, `gap_conf_*`, `confirm_frames` | mode_manager | 0.6/0.3, 0.75/0.25, 5 | rosbag 재생으로 전환 지점의 신뢰도 값을 보고 조정 |
| 모드별 lookahead, `v_max` | tracker | YAML 참고 | 저속에서 시작해 단계적으로 |

## 6. 운영

| 항목 | 위치 |
|---|---|
| maintainer, license | 각 `package.xml`, `lane_planner/setup.py` |
| 담당자 GitHub 아이디 | `.github/CODEOWNERS` |
