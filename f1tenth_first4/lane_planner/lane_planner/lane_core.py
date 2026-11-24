"""ROS 의존성 없는 차선 처리 로직.

좌표 규약
- BEV 이미지: u = 열(오른쪽 +), v = 행(아래 +)
- 차량 기준(base_link): x = 전방(+), y = 좌측(+), 단위 m
- 차량 원점은 BEV 이미지의 (origin_u, origin_v) 픽셀에 있다고 둔다.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class LaneParams:
    bev_width: int = 400                 # BEV 이미지 폭 [px]
    bev_height: int = 400                # BEV 이미지 높이 [px]
    m_per_px: float = 0.005              # BEV 1픽셀 = 몇 m (확인필요)
    origin_u: float = 200.0              # 차량 원점의 BEV 열 [px] (확인필요)
    origin_v: float = 440.0              # 차량 원점의 BEV 행 [px], 이미지 아래 밖일 수 있음 (확인필요)
    hsv_low: Tuple[int, int, int] = (15, 80, 80)    # 차선 색 하한 (확인필요: 실제 테이프 색)
    hsv_high: Tuple[int, int, int] = (40, 255, 255)  # 차선 색 상한 (확인필요)
    n_windows: int = 10
    window_margin: int = 40              # 슬라이딩 윈도우 반폭 [px]
    min_pixels: int = 30                 # 윈도우 재중심 최소 픽셀 수
    lane_width_m: float = 0.6            # 차선 폭 [m] (확인필요)
    path_step_m: float = 0.1
    path_max_m: float = 1.5
    path_min_m: float = 0.2
    single_side_penalty: float = 0.7     # 한쪽 차선만 보일 때 신뢰도 배율
    homography: List[float] = field(default_factory=lambda: [0.0] * 9)


def homography_valid(h: List[float]) -> bool:
    return len(h) == 9 and any(abs(x) > 1e-12 for x in h)


def warp_to_bev(img: np.ndarray, p: LaneParams) -> np.ndarray:
    H = np.asarray(p.homography, dtype=np.float64).reshape(3, 3)
    return cv2.warpPerspective(img, H, (p.bev_width, p.bev_height))


def lane_mask(bev_bgr: np.ndarray, p: LaneParams) -> np.ndarray:
    hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array(p.hsv_low, np.uint8), np.array(p.hsv_high, np.uint8))


def bev_to_vehicle(u: np.ndarray, v: np.ndarray, p: LaneParams):
    x = (p.origin_v - v) * p.m_per_px
    y = (p.origin_u - u) * p.m_per_px
    return x, y


def vehicle_to_bev(x: np.ndarray, y: np.ndarray, p: LaneParams):
    u = p.origin_u - y / p.m_per_px
    v = p.origin_v - x / p.m_per_px
    return u, v


def sliding_window_fit(mask: np.ndarray, p: LaneParams):
    """좌·우 차선을 u = a v^2 + b v + c 로 피팅.

    반환: (left_fit or None, right_fit or None, left_ratio, right_ratio)
    ratio = 픽셀이 충분했던 윈도우 비율 (0~1)
    """
    h, w = mask.shape[:2]
    hist = np.sum(mask[h // 2:, :] > 0, axis=0)
    mid = w // 2
    bases = []
    for lo, hi in ((0, mid), (mid, w)):
        seg = hist[lo:hi]
        bases.append(lo + int(np.argmax(seg)) if seg.size and seg.max() > 0 else None)

    ys, xs = np.nonzero(mask)
    win_h = h // p.n_windows
    fits, ratios = [], []
    for base in bases:
        if base is None:
            fits.append(None)
            ratios.append(0.0)
            continue
        cur = base
        idx_all, hits = [], 0
        for k in range(p.n_windows):
            y_hi = h - k * win_h
            y_lo = y_hi - win_h
            sel = ((ys >= y_lo) & (ys < y_hi) & (xs >= cur - p.window_margin)
                   & (xs < cur + p.window_margin))
            idx = np.flatnonzero(sel)
            if idx.size >= p.min_pixels:
                hits += 1
                cur = int(np.mean(xs[idx]))
            idx_all.append(idx)
        idx_cat = np.concatenate(idx_all) if idx_all else np.array([], dtype=int)
        if idx_cat.size >= 3 * p.min_pixels and hits >= 2:
            fits.append(np.polyfit(ys[idx_cat], xs[idx_cat], 2))
            ratios.append(hits / p.n_windows)
        else:
            fits.append(None)
            ratios.append(0.0)
    return fits[0], fits[1], ratios[0], ratios[1]


def centerline(left_fit, right_fit, left_ratio: float, right_ratio: float,
               p: LaneParams) -> Tuple[List[Tuple[float, float]], float]:
    """중심선을 차량 기준 경로점과 신뢰도로 반환."""
    if left_fit is None and right_fit is None:
        return [], 0.0
    half_w_px = 0.5 * p.lane_width_m / p.m_per_px
    xs_m = np.arange(p.path_min_m, p.path_max_m + 1e-9, p.path_step_m)
    _, vs = vehicle_to_bev(xs_m, np.zeros_like(xs_m), p)
    vs = vs[(vs >= 0) & (vs < p.bev_height)]
    if vs.size == 0:
        return [], 0.0
    if left_fit is not None and right_fit is not None:
        uc = 0.5 * (np.polyval(left_fit, vs) + np.polyval(right_fit, vs))
        conf = 0.5 * (left_ratio + right_ratio)
    elif left_fit is not None:
        uc = np.polyval(left_fit, vs) + half_w_px     # 왼쪽 선 -> 중심은 오른쪽(u+)
        conf = left_ratio * p.single_side_penalty
    else:
        uc = np.polyval(right_fit, vs) - half_w_px
        conf = right_ratio * p.single_side_penalty
    x, y = bev_to_vehicle(uc, vs, p)
    order = np.argsort(x)
    pts = [(float(x[i]), float(y[i])) for i in order]
    return pts, float(np.clip(conf, 0.0, 1.0))


def classic_plan(bev_bgr: np.ndarray, p: LaneParams):
    mask = lane_mask(bev_bgr, p)
    lf, rf, lr, rr = sliding_window_fit(mask, p)
    return centerline(lf, rf, lr, rr, p)


def lane_visibility(bev_bgr: np.ndarray, p: LaneParams) -> float:
    """학습 모델 백엔드용 신뢰도: 슬라이딩 윈도우 성공 비율을 재사용."""
    _, _, lr, rr = sliding_window_fit(lane_mask(bev_bgr, p), p)
    return float(np.clip(max(lr, rr) if min(lr, rr) == 0 else 0.5 * (lr + rr), 0.0, 1.0))


def waypoint_plan(wp_xy: Optional[Tuple[float, float]], conf: float):
    """학습 모델의 waypoint 1개를 경로로. tracker는 점이 1개면 그 점을 그대로 추종한다."""
    if wp_xy is None:
        return [], 0.0
    return [(float(wp_xy[0]), float(wp_xy[1]))], conf
