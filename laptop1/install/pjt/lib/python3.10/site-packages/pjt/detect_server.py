import rclpy
from rclpy.node import Node
import cv2
import numpy as np

import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import torch
from ultralytics import YOLO

import socket, threading
from dotenv import load_dotenv
import os

load_dotenv("/home/ssafy/Desktop/SmartFactory-Solution/laptop1/.env")
HOST = os.environ.get("HOST")


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__("realsense_yolo11_node")

        self._tcp_host = "0.0.0.0"
        self._tcp_port = 65432
        self._conn = None
        threading.Thread(target=self._start_tcp_server, daemon=True).start()
        self.get_logger().info(
            f"TCP server thread started on {self._tcp_host}:{self._tcp_port}"
        )

        self.yolo_model = YOLO(
            "/home/ssafy/Desktop/SmartFactory-Solution/laptop1/best.pt"
        )

        # Realsense camera setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # ROS publish
        self.detection_publisher = self.create_publisher(
            String, "detection_results", 10
        )
        self.image_publisher = self.create_publisher(Image, "detection_image", 10)

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_color_name(self, hsv_color):
        h, s, v = hsv_color
        if 20 < h < 120 and s < 40 and 130 < v < 230:
            return "white"
        elif (h > 150 or h < 20) and s > 200 and v > 90:
            return "red"
        elif 80 < h < 150 and s > 100 and 100 < v < 150:
            return "blue"
        return "unknown"

    def get_color_bgr(self, color_name):
        if color_name == "white":
            return (255, 255, 255)
        elif color_name == "red":
            return (0, 0, 255)
        elif color_name == "blue":
            return (255, 0, 0)
        return (0, 255, 0)  # Defalut to green for unknown colors

    def get_center_color(self, image):
        height, width = image.shape[:2]
        center_y, center_x = height // 2, width // 2
        sample_size = min(width, height) // 4

        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)

        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)

        center_region = image[start_y:end_y, start_x:end_x]
        if center_region.size == 0:
            # 빈 영역이면 기본값 반환 (예: HSV(0,0,0))
            print("center_region.size == 0 Error")
            return np.array([0, 0, 0], dtype=float)
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))

        return average_color

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # ROI 좌표
        roi_x1, roi_y1 = 100, 120
        roi_x2, roi_y2 = 510, 345

        # 원본 이미지
        color_image = np.asanyarray(color_frame.get_data())

        # 전체 이미지로 YOLO inference
        results = self.yolo_model(color_image, conf=0.7, iou=0.3)
        if len(results) == 0:
            return
        result = results[0]

        # ROI 박스 시각화
        cv2.rectangle(color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)

        self.detection_result = String()
        # 전체 바운딩 박스 순회
        for box, conf, cid in zip(
            result.boxes.xyxy, result.boxes.conf, result.boxes.cls
        ):
            x1, y1, x2, y2 = map(int, box.tolist())
            # detection 중심점 계산
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # ROI 안에 있는 detection만 처리
            if not (roi_x1 <= cx <= roi_x2 and roi_y1 <= cy <= roi_y2):
                continue

            # 좌표 클램핑
            x1, y1 = max(0, x1), max(0, y1)
            x2 = min(color_image.shape[1], x2)
            y2 = min(color_image.shape[0], y2)
            if x2 <= x1 or y2 <= y1:
                continue

            object_roi = color_image[y1:y2, x1:x2]
            if object_roi.size == 0:
                continue

            # 색상 검출
            confidence = conf.item()
            class_id = cid.item()
            center_color = self.get_center_color(object_roi)
            color_name = self.get_color_name(center_color)
            color_bgr = self.get_color_bgr(color_name)

            label = self.yolo_model.names[class_id]
            self.detection_result.data = f"{label} {color_name}"

            # 결과 그리기
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                color_image,
                f"{label}-{color_name}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        # 퍼블리시
        self.detection_publisher.publish(self.detection_result)
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.image_publisher.publish(ros_image_message)

        if self._conn:
            try:
                # 줄바꿈(\n) 단위로 RP5에서 읽기 쉽게
                payload = (self.detection_result.data + "\n").encode("utf-8")
                self._conn.sendall(payload)
            except Exception as e:
                self.get_logger().error(f"TCP send failed: {e}")

    def destory_node(self):
        self.pipeline.stop()
        super().destroy_node()

    def _start_tcp_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._tcp_host, self._tcp_port))
        srv.listen(1)
        self.get_logger().info("Waiting for RP5 connection...")
        conn, addr = srv.accept()
        self._conn = conn
        self.get_logger().info(f"RP5 connected from {addr}")


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._conn:
            node._conn.close()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
