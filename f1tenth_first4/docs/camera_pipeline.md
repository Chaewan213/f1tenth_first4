# 카메라 파이프라인: 캘리브레이션 → 데이터 → Colab 학습 → 배포

수업 실습(자동차공학실험 IRCV 자료, Part 05)의 흐름을 우리 스택에 맞춘 절차입니다.
수업 방식은 "BEV 이미지 → CNN → waypoint(wp_x, wp_y) 1개 → Pure Pursuit" 이고,
우리 스택에서는 lane_planner `backend: onnx` 가 이 역할을 합니다.

## 0. 먼저 classic 백엔드로 완주 기반 만들기

학습 모델이 없어도 `backend: classic`(HSV 임계값 + 슬라이딩 윈도우)으로 LANE 모드가 동작합니다.
아래 1단계(캘리브레이션)만 끝내면 바로 쓸 수 있고, 학습 모델의 비교 기준(베이스라인)이 됩니다.

## 1. 캘리브레이션 (실차, 1회)

1. 카메라를 최종 위치에 고정합니다. 이후 마운트를 건드리면 다시 해야 합니다.
2. 차 앞 바닥에 직사각형 꼭짓점 4개를 테이프로 표시하고, 각 점의 차량 기준 좌표(x 전방, y 좌측, m)를 잽니다. 원점은 뒷바퀴 축 중심입니다.
3. 카메라 이미지 한 장을 저장해 4개 점의 픽셀 좌표(u, v)를 읽습니다.
4. BEV 해상도를 정합니다: `m_per_px`(예: 0.005 = 1픽셀 5mm), `origin_u`, `origin_v`.
5. 실행하고 출력값을 `race_params.yaml`의 `homography`에 붙여 넣습니다.

```bash
ros2 run lane_planner calibrate_homography \
  --img "u1,v1;u2,v2;u3,v3;u4,v4" --ground "x1,y1;x2,y2;x3,y3;x4,y4" \
  --m-per-px 0.005 --origin-u 200 --origin-v 440
```

6. `publish_debug_image: true`로 켜고 `/lane/debug_bev`를 rqt_image_view로 봅니다. 차선이 평행한 직선으로 보이면 성공입니다.

## 2. Colab 실습 (시뮬레이션 데이터로 학습)

수업 레포: https://github.com/jeongtaek1m/f1tenth_gym.git

1. Colab에서 런타임 유형을 T4 GPU로 바꿉니다.
2. 수업 노트북의 환경 설정 셀로 레포를 clone하고 requirements를 설치합니다.
3. 시뮬레이션에서 BEV 이미지와 waypoint 라벨을 추출합니다. 라벨 표의 열은 `file, x, y, theta, wp_x, wp_y`입니다 (wp = 차량 기준 lookahead 지점).
4. 학습 셀을 실행하고 train/val loss를 확인합니다 (수업 예시는 Huber loss).
5. 평가: 정량(waypoint L1/L2 오차)과 정성(BEV 위에 예측점 그리기)을 모두 봅니다.
6. 시뮬레이션 closed-loop 주행으로 랩을 도는지 확인합니다.
7. 결과물(`model.pt`)을 Google Drive에 저장합니다 (`drive.mount('/content/gdrive')`).

## 3. 실차 데이터로 다시 학습 (시뮬 → 실차 차이 줄이기)

시뮬레이션 BEV와 실차 BEV(카메라 → IPM)는 질감·조명·왜곡이 다릅니다. 실차 성능이 부족하면 실차 데이터를 추가합니다.

1. 차선 구간을 천천히 수동 주행하며 rosbag을 기록합니다: `ros2 bag record /camera/image_raw /odom`
2. 각 프레임을 BEV로 바꿔 PNG로 저장합니다 (lane_core.warp_to_bev 사용).
3. 라벨 만들기 (둘 중 하나):
   - classic 백엔드의 중심선에서 lookahead 거리 지점을 자동 라벨로 쓰고, 틀린 프레임만 사람이 고칩니다.
   - 또는 중심선 위 점을 직접 클릭해 라벨링합니다.
4. 수업과 같은 표 형식(`file, wp_x, wp_y`)으로 만들고, 비슷한 연속 프레임이 train과 test에 섞이지 않게 주행 구간 단위로 나눕니다.
5. 시뮬 사전학습 모델에서 이어서 학습합니다(fine-tuning). 밝기·색 변화 augmentation을 켭니다.

## 4. 배포 (Jetson)

1. PyTorch → ONNX 내보내기 (수업 `export_resnet18_onnx.py`와 같은 방식, dummy input 크기 = 학습 입력 크기).
2. Jetson에 .onnx 복사 후 `race_params.yaml` 설정:
   - `backend: onnx`, `model_path`, `model_input_width/height`, `model_mean/std`, `model_rgb`, `model_y_sign`
3. 먼저 ONNX Runtime으로 동작을 확인하고, 속도가 부족하면 TensorRT FP16 엔진으로 바꿉니다 (수업 `trtexec --fp16`).
4. 같은 rosbag을 classic과 onnx 백엔드로 각각 재생해 `/plan/lane` 결과를 비교한 뒤 실차에 올립니다.

## 신뢰도(confidence)에 대해

학습 모델은 waypoint만 내고 신뢰도를 내지 않습니다. 그래서 onnx 백엔드에서도 슬라이딩 윈도우 검출 비율을 신뢰도로 씁니다.
차선 구간이 끝나면 이 값이 떨어지고, mode_manager가 GAP으로 넘어갑니다.
