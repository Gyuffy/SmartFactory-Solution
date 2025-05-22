import socket
import time
from dotenv import load_dotenv
import os

load_dotenv("/home/pi/gyuhwan/ws_0521/.env")
HOST = os.environ.get("HOST")
PORT = 65432


def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print(f"Connected to {HOST} : {PORT}")
            return s
        except socket.error as e:
            print(f"Connection failed: {e}, Retrying in 5 SEC...")
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
                command = data.decode("utf-8")

                if command == "1":
                    print("command 1 recieved")
                elif command == "2":
                    print("command 2 recieved")
                elif command == "3":
                    print("command 3 recieved")
                elif command == "4":
                    print("command 4 recieved")

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
