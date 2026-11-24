# 모든 로컬 플래너(lane_planner, gap_planner)가 내보내는 공통 계획 메시지.
# 규약은 docs/interfaces.md 참고.

std_msgs/Header header          # frame_id: base_link / stamp: 원본 센서(이미지·스캔) 시각
geometry_msgs/Point[] points    # 차량 기준 경로점 [m], x=전방, y=좌측(+), 가까운 순. z는 사용 안 함
float32 speed_limit             # 이 계획이 허용하는 최대 속도 [m/s]
float32 free_distance           # 경로 방향 여유거리 [m]. 모르면 -1.0 (tracker가 무시)
float32 confidence              # 0.0 ~ 1.0. mode_manager의 전환 판단에 사용

uint8 SOURCE_LANE=0
uint8 SOURCE_GAP=1
uint8 source
