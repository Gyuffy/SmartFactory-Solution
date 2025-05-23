import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, numpy as np, socket, threading
from ultralytics import YOLO
import pyrealsense2 as rs
from dotenv import load_dotenv
import os

load_dotenv("/home/ssafy/Desktop/SmartFactory-Solution/laptop1/.env")
HOST = os.environ.get("HOST")


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__("realsense_yolo11_node")
        self.yolo_model = YOLO(
            "/home/ssafy/Desktop/SmartFactory-Solution/laptop1/best.pt"
        )
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(cfg)

        self.detection_pub = self.create_publisher(String, "detection_results", 10)
        self.image_pub = self.create_publisher(Image, "detection_image", 10)
        self.bridge = CvBridge()

        self.host = HOST
        self.port = 65432
        self.conn = None
        threading.Thread(target=self._start_tcp_server, daemon=True).start()

        self.create_timer(0.1, self.timer_callback)

    def _start_tcp_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        self.get_logger().info(f"TCP server listening on {self.host}:{self.port}")
        conn, addr = srv.accept()
        self.conn = conn
        self.get_logger().info(f"RP5 connected from {addr}")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        img = np.asanyarray(color_frame.get_data())
        results = self.yolo_model(img, conf=0.7, iou=0.3)
        if len(results) == 0:
            return
        r = results[0]

        for box, conf, cid in zip(r.boxes.xyxy, r.boxes.conf, r.boxes.cls):
            x1, y1, x2, y2 = map(int, box.tolist())
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            # ROI 안에만 처리한다고 가정
            if not (100 <= cx <= 510 and 120 <= cy <= 345):
                continue

            # 패널 클래스명 & 색상 추출
            label = self.yolo_model.names[int(cid)]
            roi = img[y1:y2, x1:x2]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            avg = np.mean(hsv, axis=(0, 1))
            color = self._hsv_to_color_name(avg)

            # ROS 퍼블리시
            msg = String()
            msg.data = f"{label} {color}"
            self.detection_pub.publish(msg)

            # TCP 전송
            if self.conn:
                try:
                    self.conn.sendall((msg.data + "\n").encode("utf-8"))
                except Exception as e:
                    self.get_logger().error(f"TCP send failed: {e}")

            # 시각화
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                img,
                msg.data,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        # 이미지 퍼블리시
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_pub.publish(img_msg)

    def _hsv_to_color_name(self, hsv):
        h, s, v = hsv
        if 20 < h < 120 and s < 40 and 130 < v < 230:
            return "white"
        elif (h > 150 or h < 20) and s > 200 and v > 90:
            return "red"
        elif 80 < h < 150 and s > 100 and 100 < v < 150:
            return "blue"
        return "unknown"


def main():
    rclpy.init()
    node = RealSenseYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.conn:
            node.conn.close()
        node.pipeline.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
