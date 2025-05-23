import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import time

from dotenv import load_dotenv
import os

load_dotenv("/home/ssafy/Desktop/SmartFactory-Solution/laptop1/.env")

HOST = os.environ.get("HOST")


def start_conveyor_server(host=HOST, port=65432):
    # Create a socket (IPv4, TCP)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server is listening on {host}:{port}...")

        conn, addr = s.accept()
        print(f"Connected by {addr}")
        return conn


def handle_conveyor_client(conn, machine, status):
    if machine == "conv":
        if status == "conv_run":
            command = "1"
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command {command} to the client.")
        elif status == "conv_stop":
            command = "2"
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command {command} to the client.")

    elif machine == "seperator":
        if status == 3:
            command = "3"
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command {command} to the client.")
        elif status == 4:
            command = "4"
            conn.sendall(command.encode("utf-8"))
            print(f"Sent command {command} to the client.")
    else:
        print("Please check the machine name.")


def start_roboDK_server(host=HOST, port=20000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server is listening on {host}:{port}...")

        conn, addr = s.accept()
        print(f"Connected by {addr}")
        return conn


def handle_roboDK_client(conn, status):
    if status == "1":
        command = "1"
        conn.sendall(command.encode("utf-8"))
        print(f"Sent command {command} to the client.")
    elif status == "2":
        command = "2"
        conn.sendall(command.encode("utf-8"))
        print(f"Sent command {command} to the client.")
    elif status == "3":
        command = "3"
        conn.sendall(command.encode("utf-8"))
        print(f"Sent command {command} to the client.")
    else:
        print(f"Wrong status.")


class ObjectdetectionSubscriber(Node):
    def __init__(self):
        super().__init__("object_detection_subscriber")

        self.subscription = self.create_subscription(
            String, "/detection_results", self.listener_callback, 10
        )

        self.conv_server_conn = False
        self.roboDK_server_conn = False

        self.detection_buffer = []
        self.buffer_size = 20
        self.detection_threshold = 12

        self.get_logger().info("Object detection Subscriber has started")

        # Conveyor Server on
        server_thread = threading.Thread(target=self.start_conveyor_server_in_thread)
        server_thread.daemon = True
        server_thread.start()

        # RoboDK Server on
        server_thread = threading.Thread(target=self.start_roboDK_server_in_thread)
        server_thread.daemon = True
        server_thread.start()

    def listener_callback(self, msg):
        detection_results = msg.data
        self.get_logger().info(f"Received detection result: {detection_results}")

        detected_objects = self.parse_detection_results(detection_results)

        if "back_panel" in detected_objects:
            self.detection_buffer.append("back_panel")

        elif "board_panel" in detected_objects:
            self.detection_buffer.append("board_panel")

        else:
            self.detection_buffer.append("None")

        if len(self.detection_buffer) > self.buffer_size:
            self.detection_buffer.pop(0)

        self.check_object_detection()

    def check_object_detection(self):
        if self.detection_buffer.count("back_panel") >= self.detection_threshold:
            self.perform_task_for_object("back_panel")
        elif self.detection_buffer.count("board_panel") >= self.detection_threshold:
            self.perform_task_for_object("board_panel")

    def parse_detection_results(self, detection_results):
        try:
            if not detection_results:
                self.get_logger().warn("Empty detection results received")
                return []

            detection_objects = [obj.strip() for obj in detection_results.split(",")]
            return detection_objects

        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            return []

    def perform_task_for_object(self, object_name):
        self.get_logger().info(
            f"{object_name} detectedconsistently! Performing task..."
        )

        if object_name == "back_panel":
            self.perform_task_back_panel()
        elif object_name == "board_panel":
            self.perform_task_board_panel()

        self.detection_buffer = []

    def start_conveyor_server_in_thread(self):
        self.conv_server_conn = start_conveyor_server()

    def wait_for_conveyor_server_connection(self, timeout=10):
        start_time = time.time()

        while self.conv_server_conn is None and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for server connection...")

        if self.conv_server_conn is None:
            self.get_logger().error(
                "Failed to establish server connection within timeout."
            )
        else:
            self.get_logger().info("Conveyor server connection established")

    def start_roboDK_server_in_thread(self):
        self.roboDK_server_conn = start_roboDK_server()

    def wait_for_roboDK_server_connection(self, timeout=10):
        start_time = time.time()

        while self.roboDK_server_conn is None and time.time() - start_time < timeout:
            self.get_logger().info("Waiting for server connection...")

        if self.roboDK_server_conn is None:
            self.get_logger().error(
                "Failed to establish server connection within timeout."
            )
        else:
            self.get_logger().info("RoboDK server connection established")

    def perform_task_back_panel(self):
        self.get_logger().info("Executing task for back panel")

        self.wait_for_conveyor_server_connection()

        if self.conv_server_conn:
            handle_conveyor_client(self.conv_server_conn, "seperator", 3)

        time.sleep(5)

    def perform_task_board_panel(self):
        self.get_logger().info("Executing task for board panel")

        self.wait_for_conveyor_server_connection()
        self.wait_for_roboDK_server_connection()

        if self.conv_server_conn:
            handle_conveyor_client(self.conv_server_conn, "conv", "conv_stop")

        self.robotStatus == 1
        if self.roboDK_server_conn:
            handle_roboDK_client(self.roboDK_server_conn, self.robotStatus)
        time.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectdetectionSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
