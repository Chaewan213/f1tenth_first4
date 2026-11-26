"""학습된 waypoint 회귀 모델(BEV 이미지 -> wp_x, wp_y) 추론 래퍼.

수업 실습(Colab)에서 학습한 PyTorch 모델을 ONNX로 내보낸 뒤 사용한다.
Jetson에서는 ONNX Runtime(GPU) 또는 TensorRT 엔진으로 바꿔 끼울 수 있다.

확인필요 항목 (학습 코드와 반드시 일치시킬 것):
- input_width / input_height: 학습 시 입력 크기
- 색 순서(RGB/BGR), 정규화 mean/std, 0~1 스케일 여부
- 출력 좌표계: wp_x 전방(+), wp_y 좌측(+) 인지 -> 아니면 y_sign = -1.0
- 출력 개수: (x, y) 쌍이 몇 개인지. 수업 실차 라벨은 0.5~2.0m 7점 -> 출력 14개
- 출력 원점: 수업 라벨은 카메라 바로 아래 지면 -> 노드 파라미터 model_origin_offset_x 로 보정
"""
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


class OnnxWaypointModel:
    def __init__(self, model_path: str, input_width: int, input_height: int,
                 mean: Sequence[float], std: Sequence[float], rgb: bool = True,
                 y_sign: float = 1.0, providers: Optional[Sequence[str]] = None):
        import onnxruntime as ort  # 필요할 때만 import (classic 백엔드는 불필요)
        self.sess = ort.InferenceSession(
            model_path,
            providers=list(providers) if providers else ort.get_available_providers())
        self.input_name = self.sess.get_inputs()[0].name
        self.size = (int(input_width), int(input_height))
        self.mean = np.asarray(mean, np.float32).reshape(3, 1, 1)
        self.std = np.asarray(std, np.float32).reshape(3, 1, 1)
        self.rgb = rgb
        self.y_sign = float(y_sign)

    def preprocess(self, bev_bgr: np.ndarray) -> np.ndarray:
        img = cv2.resize(bev_bgr, self.size, interpolation=cv2.INTER_AREA)
        if self.rgb:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        x = img.astype(np.float32).transpose(2, 0, 1) / 255.0
        x = (x - self.mean) / self.std
        return x[None]

    def predict(self, bev_bgr: np.ndarray) -> List[Tuple[float, float]]:
        """모델 출력 전체를 (x, y) 쌍 목록으로 반환한다 (1점이면 길이 1, 7점이면 길이 7)."""
        out = self.sess.run(None, {self.input_name: self.preprocess(bev_bgr)})[0]
        return decode_waypoints(out, self.y_sign)


def decode_waypoints(out, y_sign: float = 1.0) -> List[Tuple[float, float]]:
    flat = np.asarray(out, dtype=np.float64).reshape(-1)
    if flat.size < 2 or flat.size % 2:
        raise ValueError(f'모델 출력 길이 {flat.size}: (x, y) 쌍이어야 합니다')
    xy = flat.reshape(-1, 2)
    return [(float(x), y_sign * float(y)) for x, y in xy]
