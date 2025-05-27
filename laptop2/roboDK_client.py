import socket
import time
import os
from dotenv import load_dotenv
import select
import main_prog

load_dotenv(r"C:\Users\SSAFY\Desktop\First_Semester_PJT\window\.env")
HOST = os.environ.get("HOST")
PORT = 20000
step = 0


def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print(f"Connected to {HOST}:{PORT}")
            return s
        except socket.error as e:
            print(f"Connect failed: {e}, retrying in 5s...")
            time.sleep(5)


def main():
    global step
    s = connect_to_server()
    buff = ""
    last_line = None

    while True:
        try:
            # 소켓에 데이터 있을 때만 읽기
            ready, _, _ = select.select([s], [], [], 0.1)
            if ready:
                data = s.recv(1024)
                if not data:
                    print("Server closed connection. 재접속...")
                    s.close()
                    s = connect_to_server()
                    buff = ""
                    last_line = None
                    continue

                buff += data.decode("utf-8")

                # 한 줄씩 파싱
                while "\n" in buff:
                    line, buff = buff.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    # 중복 커맨드 스킵
                    if line == last_line:
                        continue
                    last_line = line

                    # “job_no_object”를 받았을 때만 한 단계 실행
                    if line == "job_no_object":
                        print(f">>> Received command: {line}")
                        print(f"▶ Executing step {step}")
                        main_prog.main_process(step)
                        step += 1
                        print("⏸ Step complete, waiting for next 'job_no_object'")
                    else:
                        print(f"Unknown command, ignoring: {line}")

        except KeyboardInterrupt:
            print("프로그램 수동 종료")
            break
        except Exception as e:
            print(f"예외 발생: {e}, 재시도 전 2초 대기...")
            time.sleep(2)


if __name__ == "__main__":
    main()
