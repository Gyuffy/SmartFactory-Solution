#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import threading
import time
import sys
from dotenv import load_dotenv
import os

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl

from ultralytics import YOLO

# .env에서 HOST 읽기
load_dotenv("/home/ssafy/Desktop/SmartFactory-Solution/laptop1/.env")
HOST = os.environ.get("HOST")


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__("realsense_yolo11_node")
        # 쿨다운 설정 (초 단위)
        self.noobj_cooldown = 10.0
        self.last_noobj_time = 0.0

        # 서비스 future 초기화
        self.srv_future = None
        # pick-and-place 완료 플래그
        self.all_tasks_done = False

        # Dobot 액션 & 서비스 클라이언트 설정
        self._action_client = ActionClient(
            self,
            PointToPoint,
            "PTP_action",
            callback_group=ReentrantCallbackGroup(),
        )
        self.cli = self.create_client(SuctionCupControl, "dobot_suction_cup_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.req = SuctionCupControl.Request()

        # 작업 순서 및 인덱스 초기화
        self.tasks_list = [
            ["move", [23.8, -117.3, 60.0, -90.0], 1],
            ["move", [23.8, -117.3, -8.5, -90.0], 1],
            ["gripper", True],
            ["move", [23.8, -117.3, 60.0, -90.0], 1],
            ["move", [152.5, 30.4, 60.0, -90.0], 1],
            ["move", [152.5, 30.4, -1.9, -90.0], 1],
            ["gripper", False],
            ["move", [152.5, 30.4, 60.0, -90.0], 1],
            ["move", [18.0, -118.5, 60.0, -90.0], 1],
        ]
        self.goal_num = 0

        # 미검출 로직용 변수
        self.no_detection_since = None
        self.no_detection_actioned = False

        # TCP 서버 설정
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

        # YOLO 모델 로드
        self.yolo_model = YOLO(
            "/home/ssafy/Desktop/SmartFactory-Solution/laptop1/best.pt"
        )

        # RealSense 카메라 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # ROS 퍼블리셔
        self.detection_publisher = self.create_publisher(
            String, "detection_results", 10
        )
        self.image_publisher = self.create_publisher(Image, "detection_image", 10)
        self.bridge = CvBridge()

        # 타이머 초기화
        self.timer = None
        self.start_timer()

    def start_timer(self):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def execute(self):
        # 모든 작업 완료 시 → 다음 사이클을 위해 인덱스 리셋
        if self.goal_num > len(self.tasks_list) - 1:
            self.get_logger().info("Pick-and-place 사이클 완료, 인덱스 리셋")
            self.goal_num = 0
            # 검출 대기 상태로 돌아가기 위해 플래그 초기화
            self.no_detection_since = None
            self.no_detection_actioned = False
            return

        task = self.tasks_list[self.goal_num]
        self.get_logger().info(f"*** TASK NUM ***: {self.goal_num}  {task}")
        if task[0] == "gripper":
            # suction on/off
            self.send_request(task[1])
        else:  # "move"
            self.send_goal(task[1], task[2])

    def send_request(self, enable_suction):
        self.req.enable_suction = enable_suction
        self.srv_future = self.cli.call_async(self.req)

    def send_goal(self, target, mtype):
        goal = PointToPoint.Goal()
        goal.target_pose = target
        goal.motion_type = mtype

        self._action_client.wait_for_server()
        fut = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        fut.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return
        self.get_logger().info("Goal accepted :)")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        res = future.result()
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Action result: {res.result}")
            # 다음 작업 실행
            self.goal_num += 1
            self.execute()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback}")

    def get_color_name(self, hsv):
        h, s, v = hsv
        if 20 < h < 120 and s < 40 and 130 < v < 230:
            return "white"
        if 80 < h < 150 and s > 100 and 60 < v < 150:
            return "blue"
        return "red"

    def get_center_color(self, img):
        h, w = img.shape[:2]
        cy, cx = h // 2, w // 2
        sz = min(h, w) // 4
        reg = img[cy - sz // 2 : cy + sz // 2, cx - sz // 2 : cx + sz // 2]
        if reg.size == 0:
            return np.array([0, 0, 0], dtype=float)
        hsv = cv2.cvtColor(reg, cv2.COLOR_BGR2HSV)
        return np.mean(hsv, axis=(0, 1))

    def timer_callback(self):
        # --- 1) suction 서비스 완료 시 다음 작업 ---
        if self.srv_future is not None and self.srv_future.done():
            res = self.srv_future.result()
            self.get_logger().info(f"Service result: {res}")
            self.srv_future = None
            self.goal_num += 1
            self.execute()
            return

        # --- 3) 카메라 프레임 취득 & YOLO 검출 ---
        frames = self.pipeline.wait_for_frames()
        cf = frames.get_color_frame()
        if not cf:
            return
        img = np.asanyarray(cf.get_data())

        # ROI 설정 & 시각화
        x1, y1, x2, y2 = 160, 190, 610, 345
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
        res = self.yolo_model(img, conf=0.7, iou=0.3)[0]

        dets = []
        for b, _, cid in zip(res.boxes.xyxy, res.boxes.conf, res.boxes.cls):
            x1b, y1b, x2b, y2b = map(int, b.tolist())
            cx, cy = (x1b + x2b) // 2, (y1b + y2b) // 2
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                dets.append((x1b, y1b, x2b, y2b, int(cid)))

        now = time.time()
        # --- 4) 2초 연속 미검출 시 pick-and-place 시퀀스 시작 ---
        if not dets:
            if self.no_detection_since is None:
                self.no_detection_since = now
                self.no_detection_actioned = False
            elif (
                not self.no_detection_actioned
                and now - self.no_detection_since >= 2.0
                and now - self.last_noobj_time >= self.noobj_cooldown
            ):
                self.get_logger().info("2초 연속 미검출 → pick-and-place 시작")
                # RoboDK client에 알림
                if self._conn_20000:
                    cmd = "job_no_object\n"
                    self._conn_20000.sendall(cmd.encode())
                    self.get_logger().info(f"Sent to RoboDK: {cmd.strip()}")
                    # 마지막 전송 시간 기록
                    self.last_noobj_time = now
                self.execute()
                self.no_detection_actioned = True
            return

        # --- 검출 있으면 리셋 및 결과 퍼블리시 ---
        self.no_detection_since = None
        self.no_detection_actioned = False

        label = ""
        for x1b, y1b, x2b, y2b, cid in dets:
            roi = img[y1b:y2b, x1b:x2b]
            if roi.size == 0:
                continue
            hsv = self.get_center_color(roi)
            color = self.get_color_name(hsv)
            name = self.yolo_model.names[cid]
            label = f"{name} {color}"
            cv2.rectangle(img, (x1b, y1b), (x2b, y2b), (0, 255, 0), 2)
            cv2.putText(
                img,
                label,
                (x1b, y1b - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            break

        # ROS 퍼블리시
        msg = String()
        msg.data = label
        self.detection_publisher.publish(msg)
        rosimg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_publisher.publish(rosimg)

        # TCP 65432 전송
        if self._conn_65432 and label:
            try:
                self._conn_65432.sendall((label + "\n").encode())
            except Exception as e:
                self.get_logger().error(f"TCP 65432 send fail: {e}")

    def _start_tcp_server_65432(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._tcp_host, self._tcp_port_65432))
        srv.listen(1)
        self.get_logger().info("Waiting for RP5 on 65432…")
        conn, addr = srv.accept()
        self._conn_65432 = conn
        self.get_logger().info(f"RP5 connected from {addr}")

    def _start_tcp_server_20000(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._tcp_host, self._tcp_port_20000))
        srv.listen(1)
        self.get_logger().info("Waiting for RoboDK on 20000…")
        conn, addr = srv.accept()
        self._conn_20000 = conn
        self.get_logger().info(f"RoboDK connected from {addr}")

    def destroy_node(self):
        self.pipeline.stop()
        if self._conn_65432:
            self._conn_65432.close()
        if self._conn_20000:
            self._conn_20000.close()
        super().destroy_node()


def main(args=None):
    node = None
    initialized = False
    try:
        rclpy.init(args=args)
        initialized = True

        node = RealSenseYoloNode()
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor)

    except KeyboardInterrupt:
        pass

    finally:
        # 소켓 닫기
        if node:
            if getattr(node, "_conn_65432", None):
                node._conn_65432.close()
            if getattr(node, "_conn_20000", None):
                node._conn_20000.close()
            try:
                node.destroy_node()
            except Exception:
                pass

        # shutdown 안전 호출
        if initialized:
            try:
                rclpy.shutdown()
            except RuntimeError:
                pass

        # OpenCV 윈도우 정리
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
