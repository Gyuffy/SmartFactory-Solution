import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket, threading

HOST = os.environ.get("HOST")


class ObjectdetectionSubscriber(Node):
    def __init__(self):
        super().__init__("object_detection_subscriber")
        self.get_logger().info("Object detection Subscriber has started")

        # YOLO 토픽 구독
        self.create_subscription(
            String, "/detection_results", self.listener_callback, 10
        )

        # Conveyor 서버 소켓 설정
        self.conv_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conv_server.bind((HOST, 65432))
        self.conv_server.listen()
        self.get_logger().info(f"Conveyor server listening on {HOST}:65432")
        threading.Thread(target=self._accept_conveyor, daemon=True).start()

    def _accept_conveyor(self):
        conn, addr = self.conv_server.accept()
        self.conv_conn = conn
        self.get_logger().info(f"Conveyor client connected: {addr}")

    def listener_callback(self, msg: String):
        detected = msg.data
        if "back_panel" in detected:
            code = "BP"
        elif "board_panel" in detected:
            code = "BD"
        else:
            return

        if hasattr(self, "conv_conn") and self.conv_conn:
            try:
                self.conv_conn.sendall(code.encode("utf-8"))
                self.get_logger().info(f"Sent {code} to client")
            except Exception as e:
                self.get_logger().error(f"Send failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectdetectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "conv_conn"):
            node.conv_conn.close()
        node.conv_server.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
