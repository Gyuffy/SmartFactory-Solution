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


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__("realsense_yolo11_node")

        self.yolo_model = YOLO(
            "/home/ssafy/Desktop/SmartFactory-Solution/laptop1/best.pt"
        )

        # realsense camera setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.detection_publisher = self.create_publisher(
            String, "detection_results", 10
        )
        self.image_publisher = self.create_publisher(Image, "detection_image", 10)

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_color_name(self, hsv_color):
        h, s, v = hsv_color
        if 20 < h < 60 and s < 40 and 180 < v < 230:
            return "white"
        elif h > 150 and s > 200 and v > 150:
            return "red"
        elif 80 < h < 150 and s > 200 and 100 < v < 150:
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
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))

        return average_color

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())

        # YOLO inference
        results = self.yolo_model(color_image)
        if len(results) == 0:
            return
        result = results[0]  # 첫 번째 이미지 결과

        # 바운딩 박스/신뢰도/클래스 인덱스 꺼내기
        boxes = result.boxes.xyxy  # (N,4) tensor
        confs = result.boxes.conf  # (N,1)
        cls_ids = result.boxes.cls  # (N,1)

        self.detection_result = String()
        for box, conf, cid in zip(boxes, confs, cls_ids):
            x1, y1, x2, y2 = map(int, box.tolist())
            # 0-dim tensor → 파이썬 스칼라로 변환
            confidence = conf.item()  # 또는 float(conf)
            class_id = cid.item()  # 또는 int(cid)

            object_roi = color_image[y1:y2, x1:x2]
            center_color = self.get_center_color(object_roi)

            color_name = self.get_color_name(center_color)
            print("color_name: ", color_name)

            color_bgr = self.get_color_bgr(color_name)

            label = self.yolo_model.names[class_id]
            self.detection_result.data = label + " " + color_name

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

        self.detection_publisher.publish(self.detection_result)
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.image_publisher.publish(ros_image_message)

    def destory_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
