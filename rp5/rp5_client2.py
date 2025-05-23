import socket, time, os
from dotenv import load_dotenv

load_dotenv("/home/pi/gyuhwan/ws_0521/.env")
HOST = os.environ.get("HOST")
PORT = 65432


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


def main():
    s = connect()
    buff = ""
    try:
        while True:
            data = s.recv(1024)
            if not data:
                break
            buff += data.decode()
            # 줄 단위로 파싱
            while "\n" in buff:
                line, buff = buff.split("\n", 1)
                line = line.strip()
                # 빈 줄이나 포맷 불일치 시 건너뛰기
                if not line:
                    continue
                parts = line.split()
                if len(parts) != 2:
                    # "foo" 또는 "a b c" 등 잘못된 포맷도 건너뛰기
                    continue
                label, color = parts
                print(f">>> Detected panel: {label}, color: {color}")
                # TODO: 여기에 GPIO 제어 등 후속 동작 추가

    except KeyboardInterrupt:
        pass
    finally:
        s.close()


if __name__ == "__main__":
    main()
