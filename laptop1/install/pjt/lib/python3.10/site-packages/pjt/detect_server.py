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

        self._tcp_host = HOST
        self._tcp_port_65432 = 65432
        self._tcp_port_20000 = 20000
        self._conn_65432 = None
        self._conn_20000 = None
        threading.Thread(target=self._start_tcp_server_65432, daemon=True).start()
        threading.Thread(target=self._start_tcp_server_20000, daemon=True).start()
        self.get_logger().info(
            f"TCP server threads started on {self._tcp_host}:65432 and :20000"
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
        elif 80 < h < 150 and s > 100 and 60 < v < 150:
            return "blue"
        return "red"

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

        roi_x1, roi_y1 = 100, 120
        roi_x2, roi_y2 = 510, 345

        color_image = np.asanyarray(color_frame.get_data())

        # ROI 사각형 그리기
        cv2.rectangle(color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)

        results = self.yolo_model(color_image, conf=0.7, iou=0.3)
        if len(results) == 0:
            return
        result = results[0]

        self.detection_result = String()
        detected_label = None

        for box, conf, cid in zip(
            result.boxes.xyxy, result.boxes.conf, result.boxes.cls
        ):
            x1, y1, x2, y2 = map(int, box.tolist())
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            if not (roi_x1 <= cx <= roi_x2 and roi_y1 <= cy <= roi_y2):
                continue

            object_roi = color_image[y1:y2, x1:x2]
            if object_roi.size == 0:
                continue

            center_color = self.get_center_color(object_roi)
            color_name = self.get_color_name(center_color)
            class_id = cid.item()
            label = self.yolo_model.names[class_id]
            label_with_color = f"{label} {color_name}"

            self.detection_result.data = label_with_color
            detected_label = label

            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                color_image,
                label_with_color,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

            break

        self.detection_publisher.publish(self.detection_result)
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.image_publisher.publish(ros_image_message)

        if self._conn_65432 and self.detection_result.data:
            try:
                payload_65432 = (self.detection_result.data + "\n").encode("utf-8")
                self._conn_65432.sendall(payload_65432)
            except Exception as e:
                self.get_logger().error(f"TCP send failed (65432): {e}")

        if self._conn_20000 and detected_label:

            try:
                payload_65432 = (self.detection_result.data + "\n").encode("utf-8")
                self._conn_20000.sendall(payload_65432)
            except Exception as e:
                self.get_logger().error(f"TCP send failed (20000): {e}")

            # try:
            #     if detected_label == "back_panel":
            #         cmd = "job_back_panel\n"
            #     elif detected_label == "board_panel":
            #         cmd = "job_board_panel\n"
            #     else:
            #         cmd = None

            #     if cmd:
            #         self._conn_20000.sendall(cmd.encode("utf-8"))
            # except Exception as e:
            #     self.get_logger().error(f"TCP send failed (20000): {e}")

    def _start_tcp_server_65432(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._tcp_host, self._tcp_port_65432))
        srv.listen(1)
        self.get_logger().info("Waiting for RP5 connection on 65432...")
        conn, addr = srv.accept()
        self._conn_65432 = conn
        self.get_logger().info(f"RP5 connected from {addr} (65432)")

    def _start_tcp_server_20000(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._tcp_host, self._tcp_port_20000))
        srv.listen(1)
        self.get_logger().info("Waiting for RoboDK client connection on 20000...")
        conn, addr = srv.accept()
        self._conn_20000 = conn
        self.get_logger().info(f"Client connected from {addr} (20000)")

    def destory_node(self):
        self.pipeline.stop()
        # 커넥션 정리
        if self._conn_65432:
            self._conn_65432.close()
        if self._conn_20000:
            self._conn_20000.close()
        super().destroy_node()


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
