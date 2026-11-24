"""lane_planner 노드: 카메라 이미지 -> BEV -> 차선 경로 -> /plan/lane (LocalPlan)

백엔드
- classic: HSV 임계값 + 슬라이딩 윈도우로 중심선을 만든다 (모델 없이 바로 동작)
- onnx:    수업에서 학습한 waypoint 회귀 모델로 점 1개를 만든다
이 노드는 /drive를 직접 발행하지 않는다.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from race_msgs.msg import LocalPlan
from sensor_msgs.msg import Image

from lane_planner import lane_core
from lane_planner.lane_core import LaneParams


class LanePlannerNode(Node):
    def __init__(self):
        super().__init__('lane_planner')
        d = LaneParams()
        g = self.declare_parameter
        self.backend = g('backend', 'classic').value
        self.input_is_bev = g('input_is_bev', False).value
        self.speed_limit = g('speed_limit', 1.5).value
        self.publish_debug = g('publish_debug_image', False).value
        image_topic = g('image_topic', '/camera/image_raw').value

        self.p = LaneParams(
            bev_width=g('bev_width', d.bev_width).value,
            bev_height=g('bev_height', d.bev_height).value,
            m_per_px=g('m_per_px', d.m_per_px).value,
            origin_u=g('origin_u', d.origin_u).value,
            origin_v=g('origin_v', d.origin_v).value,
            hsv_low=tuple(g('hsv_low', list(d.hsv_low)).value),
            hsv_high=tuple(g('hsv_high', list(d.hsv_high)).value),
            n_windows=g('n_windows', d.n_windows).value,
            window_margin=g('window_margin', d.window_margin).value,
            min_pixels=g('min_pixels', d.min_pixels).value,
            lane_width_m=g('lane_width_m', d.lane_width_m).value,
            path_step_m=g('path_step_m', d.path_step_m).value,
            path_max_m=g('path_max_m', d.path_max_m).value,
            path_min_m=g('path_min_m', d.path_min_m).value,
            single_side_penalty=g('single_side_penalty', d.single_side_penalty).value,
            homography=list(g('homography', [0.0] * 9).value),
        )

        self.model = None
        if self.backend == 'onnx':
            from lane_planner.waypoint_model import OnnxWaypointModel
            self.model = OnnxWaypointModel(
                model_path=g('model_path', '').value,
                input_width=g('model_input_width', 224).value,
                input_height=g('model_input_height', 224).value,
                mean=g('model_mean', [0.485, 0.456, 0.406]).value,
                std=g('model_std', [0.229, 0.224, 0.225]).value,
                rgb=g('model_rgb', True).value,
                y_sign=g('model_y_sign', 1.0).value,
            )
        elif self.backend != 'classic':
            self.get_logger().warn(f"backend '{self.backend}' 미지원, classic 사용")
            self.backend = 'classic'

        if not self.input_is_bev and not lane_core.homography_valid(self.p.homography):
            self.get_logger().error(
                'homography가 비어 있습니다(확인필요). calibrate_homography로 구한 값을 '
                'race_params.yaml에 넣으세요. 그 전까지 confidence=0으로 발행합니다.')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(LocalPlan, '/plan/lane', 10)
        self.pub_dbg = self.create_publisher(Image, '/lane/debug_bev', 1) if self.publish_debug else None
        self.sub = self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'lane_planner started (backend={self.backend}, image={image_topic})')

    def on_image(self, msg: Image):
        plan = LocalPlan()
        plan.header.stamp = msg.header.stamp
        plan.header.frame_id = 'base_link'
        plan.source = LocalPlan.SOURCE_LANE
        plan.free_distance = -1.0
        plan.speed_limit = float(self.speed_limit)

        if not self.input_is_bev and not lane_core.homography_valid(self.p.homography):
            plan.confidence = 0.0
            self.pub.publish(plan)
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        bev = img if self.input_is_bev else lane_core.warp_to_bev(img, self.p)
        if self.input_is_bev and bev.shape[:2] != (self.p.bev_height, self.p.bev_width):
            bev = cv2.resize(bev, (self.p.bev_width, self.p.bev_height))

        if self.backend == 'onnx':
            conf = lane_core.lane_visibility(bev, self.p)
            pts, conf = lane_core.waypoint_plan(self.model.predict(bev), conf)
        else:
            pts, conf = lane_core.classic_plan(bev, self.p)

        for x, y in pts:
            plan.points.append(Point(x=x, y=y, z=0.0))
        plan.confidence = float(conf)
        self.pub.publish(plan)

        if self.pub_dbg is not None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(self.draw_debug(bev, pts), 'bgr8'))

    def draw_debug(self, bev, pts):
        out = bev.copy()
        for x, y in pts:
            u, v = lane_core.vehicle_to_bev(np.array([x]), np.array([y]), self.p)
            cv2.circle(out, (int(u[0]), int(v[0])), 3, (0, 0, 255), -1)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = LanePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
