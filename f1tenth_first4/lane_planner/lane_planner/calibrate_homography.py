"""카메라 이미지 -> BEV 호모그래피 계산 도구.

사용법
1) 차를 평평한 바닥에 두고, 바닥에 4개 점을 표시한다 (예: 테이프 십자).
2) 각 점의 차량 기준 좌표(x 전방, y 좌측, m)를 줄자로 잰다.
3) 카메라 이미지에서 같은 4개 점의 픽셀 좌표(u, v)를 읽는다.
4) 실행:
   ros2 run lane_planner calibrate_homography \
     --img "u1,v1;u2,v2;u3,v3;u4,v4" --ground "x1,y1;x2,y2;x3,y3;x4,y4" \
     --m-per-px 0.005 --origin-u 200 --origin-v 440
5) 출력된 homography 값을 race_bringup/config/race_params.yaml 에 붙여 넣는다.
"""
import argparse

import cv2
import numpy as np


def parse_pts(s: str) -> np.ndarray:
    pts = [tuple(float(v) for v in p.split(',')) for p in s.split(';') if p.strip()]
    if len(pts) != 4:
        raise ValueError('점은 정확히 4개여야 합니다')
    return np.asarray(pts, np.float32)


def compute_homography(img_pts, ground_pts, m_per_px, origin_u, origin_v):
    u = origin_u - ground_pts[:, 1] / m_per_px
    v = origin_v - ground_pts[:, 0] / m_per_px
    bev_pts = np.stack([u, v], axis=1).astype(np.float32)
    return cv2.getPerspectiveTransform(img_pts.astype(np.float32), bev_pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--img', required=True)
    ap.add_argument('--ground', required=True)
    ap.add_argument('--m-per-px', type=float, required=True)
    ap.add_argument('--origin-u', type=float, required=True)
    ap.add_argument('--origin-v', type=float, required=True)
    a = ap.parse_args()
    H = compute_homography(parse_pts(a.img), parse_pts(a.ground), a.m_per_px, a.origin_u, a.origin_v)
    vals = ', '.join(f'{x:.8g}' for x in H.reshape(-1))
    print(f'homography: [{vals}]')


if __name__ == '__main__':
    main()
