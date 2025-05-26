import socket
import time
import os
from dotenv import load_dotenv
import select
import main_prog

load_dotenv(r"C:\Users\SSAFY\Desktop\First_Semester_PJT\window\.env")
HOST = os.environ.get("HOST")
PORT = 20000


def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print(f"Connected to {HOST}:{PORT}")
            return s
        except socket.error as e:
            print(f"Connect failed: {e}")
            time.sleep(5)


def main():
    while True:
        s = connect_to_server()
        buff = ""
        try:
            while True:
                # select를 사용해 데이터가 올 때만 읽음
                ready, _, _ = select.select([s], [], [], 0.1)
                if ready:
                    data = s.recv(1024)
                    if not data:
                        print("Server closed connection.")
                        break
                    buff += data.decode("utf-8")

                    # 줄 단위로 파싱
                    while "\n" in buff:
                        line, buff = buff.split("\n", 1)
                        # buff = buff.split("Invalid message: ", 1)
                        line = line.strip()
                        if not line:
                            continue

                        # "panel color" 구조일 때만 처리
                        parts = line.split()
                        if len(parts) != 2:
                            print("Invalid message:", line)
                            continue
                        label, color = parts
                        print(f">>> Detected panel: {label}, color: {color}")

                        # 분기 처리: label에 따라 동작
                        if label == "back_panel":
                            print("back_panel")
                            main_prog.main_process()
                        elif label == "board_panel":
                            main_prog.main_process()
                        else:
                            print(f"Unknown label: {label}")

                # 혹시 추가 명령어 수신 루프(원하면 아래처럼, 아니면 삭제)
                # (명령 프롬프트 방식이 필요 없다면 이 부분 생략 가능)
                # command = input("Enter command (job1/job2/exit/...): ")
                # if command == "exit":
                #     break
        except KeyboardInterrupt:
            print("프로그램 수동 종료")
            break
        except Exception as e:
            print(f"예외 발생: {e}")
            time.sleep(2)
        finally:
            s.close()
            print("연결 종료. 재접속 시도...")


if __name__ == "__main__":
    main()
