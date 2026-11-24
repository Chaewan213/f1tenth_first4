"""ROS 없이 실행되는 lane_core 단위 테스트: python3 -m pytest lane_planner/test"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lane_planner import lane_core  # noqa: E402
from lane_planner.calibrate_homography import compute_homography  # noqa: E402
from lane_planner.lane_core import LaneParams  # noqa: E402


def synthetic_bev(p, offset_m=0.0, left=True, right=True):
    """차량 중심에서 offset_m 만큼 좌측으로 밀린 직선 차선(노란색) BEV 이미지."""
    img = np.full((p.bev_height, p.bev_width, 3), 90, np.uint8)
    yellow = (0, 220, 230)  # BGR
    for side, on in ((+1, left), (-1, right)):
        if not on:
            continue
        y = offset_m + side * p.lane_width_m / 2
        u = int(round(p.origin_u - y / p.m_per_px))
        cv2.line(img, (u, 0), (u, p.bev_height - 1), yellow, 6)
    return img


def test_centerline_straight():
    p = LaneParams()
    pts, conf = lane_core.classic_plan(synthetic_bev(p), p)
    assert conf > 0.8
    assert len(pts) > 3
    assert all(abs(y) < 0.03 for _, y in pts)
    assert all(pts[i][0] < pts[i + 1][0] for i in range(len(pts) - 1))


def test_centerline_offset_left():
    p = LaneParams()
    pts, _ = lane_core.classic_plan(synthetic_bev(p, offset_m=0.1), p)
    assert abs(np.mean([y for _, y in pts]) - 0.1) < 0.03


def test_single_side_uses_half_width():
    p = LaneParams()
    pts, conf = lane_core.classic_plan(synthetic_bev(p, right=False), p)
    assert 0.0 < conf <= p.single_side_penalty + 1e-9
    assert abs(np.mean([y for _, y in pts])) < 0.03


def test_no_lane():
    p = LaneParams()
    img = np.full((p.bev_height, p.bev_width, 3), 90, np.uint8)
    pts, conf = lane_core.classic_plan(img, p)
    assert pts == [] and conf == 0.0


def test_homography_roundtrip():
    p = LaneParams()
    img_pts = np.array([[100, 400], [540, 400], [400, 250], [240, 250]], np.float32)
    ground = np.array([[0.4, 0.3], [0.4, -0.3], [1.2, -0.3], [1.2, 0.3]], np.float32)
    H = compute_homography(img_pts, ground, p.m_per_px, p.origin_u, p.origin_v)
    bev = cv2.perspectiveTransform(img_pts.reshape(-1, 1, 2), H).reshape(-1, 2)
    x, y = lane_core.bev_to_vehicle(bev[:, 0], bev[:, 1], p)
    assert np.allclose(np.stack([x, y], 1), ground, atol=1e-4)


def test_waypoint_plan():
    assert lane_core.waypoint_plan((0.9, 0.2), 0.8) == ([(0.9, 0.2)], 0.8)
    assert lane_core.waypoint_plan(None, 0.8) == ([], 0.0)
