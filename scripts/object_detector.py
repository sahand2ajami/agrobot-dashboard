#!/usr/bin/env python3
"""
Person detector node: YOLOv8 (COCO) + ZED 2i depth for distance estimation.

Subscribes (ZED ROS2 driver topics — adjust namespace to match your launch):
  /zed_front/zed_node/rgb/image_rect_color   (sensor_msgs/Image, BEST_EFFORT)
  /zed_front/zed_node/depth/depth_registered (sensor_msgs/Image, 32FC1, metres, BEST_EFFORT)

Publishes:
  /object_detection/image_annotated  (sensor_msgs/Image, bgr8)
  /object_detection/detections       (std_msgs/String, JSON)

Also writes /tmp/object_detections.json for the dashboard /api/detection/data endpoint.

Note: the dashboard's on-demand YOLO (serve.py) uses the ZED pyzed SDK directly and
does not need this ROS node. This node is an optional standalone path for when the
ZED ROS2 driver (zed-ros2-wrapper) is running.
"""
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

DETECTIONS_FILE = "/tmp/object_detections.json"
PERSON_CLASS = 0

# Annotation colours (BGR)
BOX_COLOUR  = (0, 200, 50)
TEXT_COLOUR = (0, 0, 0)


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('model',               'yolov8s.pt')
        self.declare_parameter('confidence',          0.5)
        self.declare_parameter('throttle_hz',         10.0)
        self.declare_parameter('use_depth',           True)
        self.declare_parameter('depth_sample_radius', 5)

        model_name   = self.get_parameter('model').value
        self._conf   = self.get_parameter('confidence').value
        throttle_hz  = self.get_parameter('throttle_hz').value
        self._use_depth   = self.get_parameter('use_depth').value
        self._depth_r     = self.get_parameter('depth_sample_radius').value

        self._min_interval = 1.0 / max(throttle_hz, 0.1)
        self._last_infer   = 0.0

        self.get_logger().info(f'Loading YOLO model: {model_name}')
        self._model = YOLO(model_name)
        # Warm-up pass so first real frame isn't slow
        self._model(np.zeros((480, 640, 3), dtype=np.uint8),
                    classes=[PERSON_CLASS], verbose=False)
        self.get_logger().info('YOLO ready')

        # Depth frame cache (written by depth callback, read by colour callback)
        self._depth_lock = threading.Lock()
        self._depth_arr  = None   # numpy uint16 H×W (millimetres)
        self._depth_ts   = 0.0    # monotonic time of last depth frame

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._img_pub = self.create_publisher(Image,  '/object_detection/image_annotated', 1)
        self._det_pub = self.create_publisher(String, '/object_detection/detections', 10)

        colour_topic = self.declare_parameter(
            'colour_topic', '/zed_front/zed_node/rgb/image_rect_color').value
        depth_topic  = self.declare_parameter(
            'depth_topic',  '/zed_front/zed_node/depth/depth_registered').value

        self.create_subscription(Image, colour_topic, self._colour_cb, best_effort)
        self.get_logger().info(f'Subscribing to colour: {colour_topic}')

        if self._use_depth:
            self.create_subscription(Image, depth_topic, self._depth_cb, best_effort)
            self.get_logger().info(f'Subscribing to depth: {depth_topic}')
        else:
            self.get_logger().info('Depth estimation disabled (use_depth:=false)')

        self.get_logger().info('Object detector ready — subscribing to camera topics')

    # ------------------------------------------------------------------ #
    # Depth callback                                                       #
    # ------------------------------------------------------------------ #
    def _depth_cb(self, msg: Image):
        # ZED depth is 32FC1 (float32, metres). RealSense was 16UC1 (uint16, mm).
        dtype = np.float32 if '32FC1' in msg.encoding or 'float' in msg.encoding.lower() \
                else np.uint16
        arr = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.width)
        with self._depth_lock:
            self._depth_arr = arr
            self._depth_ts  = time.monotonic()

    # ------------------------------------------------------------------ #
    # Colour callback — runs inference                                     #
    # ------------------------------------------------------------------ #
    def _colour_cb(self, msg: Image):
        now = time.monotonic()
        if now - self._last_infer < self._min_interval:
            return
        self._last_infer = now

        bgr = self._ros_image_to_bgr(msg)
        if bgr is None:
            return

        with self._depth_lock:
            depth_arr = self._depth_arr.copy() if self._depth_arr is not None else None
            depth_age = now - self._depth_ts if self._depth_arr is not None else 999.0

        results = self._model(bgr, classes=[PERSON_CLASS], conf=self._conf, verbose=False)

        detections = []
        annotated  = bgr.copy()

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])

                distance_m = None
                if depth_arr is not None and depth_age < 0.5:
                    distance_m = self._estimate_distance(depth_arr, x1, y1, x2, y2)

                label = f'person {conf:.0%}'
                if distance_m is not None:
                    label += f'  {distance_m:.1f} m'

                self._draw_box(annotated, x1, y1, x2, y2, label)

                detections.append({
                    'label':       'person',
                    'confidence':  round(conf, 3),
                    'distance_m':  round(distance_m, 2) if distance_m is not None else None,
                    'bbox':        [x1, y1, x2, y2],
                })

        self._publish_image(msg, annotated)
        self._publish_detections(detections)

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _ros_image_to_bgr(msg: Image):
        try:
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            ch  = max(1, min(len(raw) // (msg.height * msg.width), 4)) if msg.width and msg.height else 3
            arr = raw.reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
            enc = msg.encoding.lower()
            return arr[:, :, ::-1].copy() if enc in ('rgb8', 'rgb') else arr.copy()
        except Exception:
            return None

    def _estimate_distance(self, depth_arr: np.ndarray, x1, y1, x2, y2) -> float | None:
        """Median depth at bbox centre.

        ZED depth is float32 in metres (NaN/inf for invalid pixels).
        Legacy uint16 depth (RealSense) is in mm and is converted to metres.
        """
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        r      = self._depth_r
        h, w   = depth_arr.shape
        patch  = depth_arr[max(0, cy - r):min(h, cy + r + 1),
                           max(0, cx - r):min(w, cx + r + 1)]
        if depth_arr.dtype == np.float32:
            valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 40.0)]
            return float(np.median(valid)) if len(valid) > 0 else None
        else:   # uint16 mm (legacy)
            valid = patch[patch > 0]
            return float(np.median(valid)) / 1000.0 if len(valid) > 0 else None

    @staticmethod
    def _draw_box(img, x1, y1, x2, y2, label: str):
        cv2.rectangle(img, (x1, y1), (x2, y2), BOX_COLOUR, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(img, (x1, y1 - th - 8), (x1 + tw + 6, y1), BOX_COLOUR, -1)
        cv2.putText(img, label, (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT_COLOUR, 2)

    def _publish_image(self, src_msg: Image, bgr: np.ndarray):
        out = Image()
        out.header   = src_msg.header
        out.height   = bgr.shape[0]
        out.width    = bgr.shape[1]
        out.encoding = 'bgr8'
        out.step     = bgr.shape[1] * 3
        out.data     = bgr.tobytes()
        self._img_pub.publish(out)

    def _publish_detections(self, detections: list):
        payload = {
            'ts':         time.time(),
            'count':      len(detections),
            'detections': detections,
        }
        data = json.dumps(payload)
        self._det_pub.publish(String(data=data))
        try:
            Path(DETECTIONS_FILE).write_text(data)
        except Exception:
            pass


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
