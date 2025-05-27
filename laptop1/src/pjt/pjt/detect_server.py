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

        # ==== pick-and-place task list 동적 생성용 설정 ====
        self.base_tasks_list = [
            ["move", [23.8, -120.3, 60.0, -90.0], 1],
            ["move", [23.8, -120.3, -11.5, -90.0], 1],
            ["gripper", True],
            ["move", [23.8, -120.3, 60.0, -90.0], 1],
            ["move", [152.5, 30.4, 60.0, -90.0], 1],
            ["move", [152.5, 30.4, -1.9, -90.0], 1],
            ["gripper", False],
            ["move", [152.5, 30.4, 60.0, -90.0], 1],
            ["move", [18.0, -118.5, 60.0, -90.0], 1],
        ]
        self.loop_count = 0
        self.x_offsets = [0.0, 36.5, 77.0]
        self.y_offsets = [0.0, -75.0]
        self.update_tasks_list()

        # ==== cooldown 설정 ====
        self.noobj_cooldown = 10.0
        self.last_noobj_time = 0.0

        # 서비스 & 액션 클라이언트
        self._action_client = ActionClient(
            self, PointToPoint, "PTP_action", callback_group=ReentrantCallbackGroup()
        )
        self.cli = self.create_client(SuctionCupControl, "dobot_suction_cup_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.req = SuctionCupControl.Request()

        # 인덱스 초기화
        self.goal_num = 0
        self.no_detection_since = None
        self.no_detection_actioned = False

        # TCP 서버 스레드
        self._tcp_host = HOST
        self._tcp_port_65432 = 65432
        self._tcp_port_20000 = 20000
        self._conn_65432 = None
        self._conn_20000 = None
        threading.Thread(target=self._start_tcp_server_65432, daemon=True).start()
        threading.Thread(target=self._start_tcp_server_20000, daemon=True).start()
        self.get_logger().info(f"TCP servers on {HOST}:65432 & 20000 started")

        # YOLO, RealSense, ROS 퍼블리셔 설정
        self.yolo_model = YOLO(
            "/home/ssafy/Desktop/SmartFactory-Solution/laptop1/best.pt"
        )
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(cfg)
        self.detection_publisher = self.create_publisher(
            String, "detection_results", 10
        )
        self.image_publisher = self.create_publisher(Image, "detection_image", 10)
        self.bridge = CvBridge()

        # 타이머 시작
        self.timer = None
        self.start_timer()

    def update_tasks_list(self):
        """loop_count에 맞춰 self.tasks_list를 재생성.
        ['gripper', True] 이후의 항목은 변경하지 않음."""
        x_idx = self.loop_count % len(self.x_offsets)
        y_idx = (self.loop_count // len(self.x_offsets)) % len(self.y_offsets)
        x_off = self.x_offsets[x_idx]
        y_off = self.y_offsets[y_idx]

        new_list = []
        before_gripper_true = True
        for task in self.base_tasks_list:
            if before_gripper_true and task[0] == "move":
                bx, by, bz, br = task[1]
                new_list.append(["move", [bx + x_off, by + y_off, bz, br], task[2]])
            else:
                # gripper 명령 포함 이후 모든 항목은 원본 그대로
                new_list.append(task.copy())
                # ['gripper', True]를 만난 뒤부터는 offset 적용 안 함
                if task[0] == "gripper" and task[1] is True:
                    before_gripper_true = False

        self.tasks_list = new_list
        self.get_logger().info(
            f"◆ tasks_list updated: loop={self.loop_count+1} "
            f"(x+{x_off:.1f}, y{y_off:+.1f}),"
            f" after-gripper-fixed"
        )

    def start_timer(self):
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def execute(self):
        # 한 사이클 끝났으면 loop_count 증가 + tasks_list 갱신
        if self.goal_num > len(self.tasks_list) - 1:
            self.get_logger().info("Cycle complete → next loop")
            self.goal_num = 0
            self.loop_count += 1
            self.update_tasks_list()
            self.no_detection_since = None
            self.no_detection_actioned = False
            self.start_timer()
            return

        task = self.tasks_list[self.goal_num]
        self.get_logger().info(f"*** TASK {self.goal_num}: {task}")
        if task[0] == "gripper":
            self.send_request(task[1])
        else:
            self.send_goal(task[1], task[2])

    def send_request(self, enable_suction):
        self.req.enable_suction = enable_suction
        fut = self.cli.call_async(self.req)
        fut.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        res = future.result()
        self.get_logger().info(f"Suction result: {res}")
        self.goal_num += 1
        self.execute()

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
            self.get_logger().info("Goal rejected")
            return
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        res = future.result()
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Action done")
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
            return np.zeros(3)
        hsv = cv2.cvtColor(reg, cv2.COLOR_BGR2HSV)
        return np.mean(hsv, axis=(0, 1))

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        cf = frames.get_color_frame()
        if not cf:
            return
        img = np.asanyarray(cf.get_data())
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
        if not dets:
            if self.no_detection_since is None:
                self.no_detection_since = now
                self.no_detection_actioned = False
            elif (
                not self.no_detection_actioned
                and now - self.no_detection_since >= 2.0
                and now - self.last_noobj_time >= self.noobj_cooldown
            ):
                self.get_logger().info("2s no-detect → start PnP")
                if self._conn_20000:
                    cmd = "job_no_object\n"
                    self._conn_20000.sendall(cmd.encode())
                    self.get_logger().info(f"Sent: {cmd.strip()}")
                    self.last_noobj_time = now
                if self.timer:
                    self.timer.cancel()
                self.execute()
                self.no_detection_actioned = True
            return

        # detection reset & publish
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

        msg = String()
        msg.data = label
        self.detection_publisher.publish(msg)
        rosimg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_publisher.publish(rosimg)

        if self._conn_65432 and label:
            try:
                self._conn_65432.sendall((label + "\n").encode())
            except Exception as e:
                self.get_logger().error(f"TCP65432 send fail: {e}")

    def _start_tcp_server_65432(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._tcp_host, self._tcp_port_65432))
        srv.listen(1)
        self.get_logger().info("Waiting RP5 on 65432…")
        conn, addr = srv.accept()
        self._conn_65432 = conn
        self.get_logger().info(f"RP5 connected: {addr}")

    def _start_tcp_server_20000(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self._tcp_host, self._tcp_port_20000))
        srv.listen(1)
        self.get_logger().info("Waiting RoboDK on 20000…")
        conn, addr = srv.accept()
        self._conn_20000 = conn
        self.get_logger().info(f"RoboDK connected: {addr}")

    def destroy_node(self):
        self.pipeline.stop()
        if self._conn_65432:
            self._conn_65432.close()
        if self._conn_20000:
            self._conn_20000.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
