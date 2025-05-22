import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import time

HOST = '127.0.0.1'
PORT = 20000

def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST,PORT))
            print(f"Connected to {HOST} : {PORT}")
            return s
        except socket.error as e:
            print(f"COnnection failed: {e}, Retrying in 5 SEC...")
            s.close()
            time.sleep(5)

try:
    while True:
        # Try to connect to the server
        s = connect_to_server()

        while True:
            try:
                data = s.recv(1024)
                if not data:
                    break
                command = data.decode('utf-8')

                if command == '1':
                    print("command 1 recieved")
                elif command == '2':
                    print("command 2 recieved")
                else:
                    print("Unknow command received")
                    time.sleep(0.1)

            except socket.error as e:
                print(f"Socket error: {e}. Reconnecting...")
                break

        s.close()

except KeyboardInterrupt():
    print("program terminated")

except Exception as e:
    print(f"An error occurred: {e}")
finally:
    print("Resource released and motor stopped")


    
