import socket
import time
import os
from dotenv import load_dotenv
import gpiod
import threading
import select

# ── 환경 설정 ───────────────────────────────────────────────────────────────
load_dotenv("/home/pi/gyuhwan/ws_0521/.env")
HOST = os.environ.get("HOST")
PORT = 65432

# ── GPIO 핀 번호 ────────────────────────────────────────────────────────────
DIR_PIN = 17
STEP_PIN = 27
ENABLE_PIN = 22
SERVO_PIN = 18

# ── Servo angle ──────────────────────────────────────────────────────────
CENTER_ANGLE = 135
RIGHT_ANGLE = 175
LEFT_ANGLE = 95

# ── 전역 제어 변수 ──────────────────────────────────────────────────────────
motor_running = False  # 모터 구동 플래그
step_delay = 0.0005  # 펄스 딜레이 (초) → 속도 조절: 값을 키우면 속도 ↓

# ── GPIO 초기화 ────────────────────────────────────────────────────────────
chip = gpiod.Chip("gpiochip0")
dir_line = chip.get_line(DIR_PIN)
step_line = chip.get_line(STEP_PIN)
enable_line = chip.get_line(ENABLE_PIN)
servo_line = chip.get_line(SERVO_PIN)

dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
enable_line.request(consumer="enbl", type=gpiod.LINE_REQ_DIR_OUT)
servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)


# ── 모터 스텝 함수 ──────────────────────────────────────────────────────────
def step_motor():
    global motor_running, step_delay
    while True:
        if motor_running:
            step_line.set_value(1)
            time.sleep(step_delay)
            step_line.set_value(0)
            time.sleep(step_delay)
        else:
            # 모터 정지 중에는 잠시 대기
            time.sleep(0.01)


# ── 서보 모터 함수 ──────────────────────────────────────────────────────────
def set_servo(angle):
    pulse_width = (angle / 270) * (0.0025 - 0.0005) + 0.0005
    for _ in range(10):
        servo_line.set_value(1)
        time.sleep(pulse_width)
        servo_line.set_value(0)
        time.sleep(0.02 - pulse_width)


# ── 서버 연결 재시도 함수 ───────────────────────────────────────────────────
def connect():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print(f"Connected to {HOST}:{PORT}")
            return s
        except Exception as e:
            print("Connect failed:", e)
            time.sleep(5)


# ── 메인 로직 ─────────────────────────────────────────────────────────────
def main():
    global motor_running

    # 스레드는 한 번만 띄워 두고, daemon=True 로 설정해서 메인 종료 시 자동 종료
    t = threading.Thread(target=step_motor, daemon=True)
    t.start()

    s = connect()
    buff = ""
    last_detect_time = time.time()
    TIMEOUT = 5.0  # 감지 없을 때 자동 정지 기준(초)

    try:
        while True:
            # 데이터 수신 대기: 타임아웃 0.1초
            ready, _, _ = select.select([s], [], [], 0.1)
            if ready:
                data = s.recv(1024)
                if not data:
                    print("Server closed connection.")
                    break

                buff += data.decode()
                # 줄 단위 파싱
                while "\n" in buff:
                    line, buff = buff.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) != 2:
                        continue
                    label, color = parts
                    print(f">>> Detected panel: {label}, color: {color}")

                    # 백패널 감지 시 모터 구동
                    if label == "back_panel" and "board_panel":
                        motor_running = True
                        dir_line.set_value(0)  # CCW 방향
                        enable_line.set_value(0)  # 모터 드라이버 활성화

                        if color == "blue":
                            set_servo(LEFT_ANGLE)

                        else:
                            set_servo(RIGHT_ANGLE)

                    else:
                        motor_running = False
                        enable_line.set_value(1)  # 모터 드라이버 비활성화

                    last_detect_time = time.time()
            else:
                # 아무 데이터도 안 왔을 때 타임아웃 경과하면 정지
                if motor_running and (time.time() - last_detect_time) > TIMEOUT:
                    print("No detection timeout → Stopping motor")
                    motor_running = False
                    enable_line.set_value(1)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        s.close()
        # GPIO 릴리즈 전 모터 정지
        motor_running = False
        time.sleep(0.1)
        enable_line.set_value(1)
        dir_line.release()
        step_line.release()
        enable_line.release()
        servo_line.realease()


if __name__ == "__main__":
    main()
