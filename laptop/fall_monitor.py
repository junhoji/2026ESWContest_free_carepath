"""환자 태그의 낙상 신호를 수신해 알람을 띄우는 모니터링 프로그램."""
import socket
import sys
import time
import subprocess
from datetime import datetime

UDP_PORT = 5005
DUPLICATE_WINDOW = 5.0    # 같은 낙상에 대한 중복 알람을 억제할 시간 (초)


def alarm_sound():
    """실행 환경에 맞는 경고음을 재생한다."""
    try:
        import winsound
        for _ in range(3):
            winsound.Beep(1200, 250)
            winsound.Beep(900, 250)
        return
    except ImportError:
        pass

    try:
        subprocess.Popen(
            ["paplay", "/usr/share/sounds/freedesktop/stereo/alarm-clock-elapsed.oga"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return
    except FileNotFoundError:
        pass

    # 사운드 재생 수단이 없으면 터미널 벨로 대체
    for _ in range(6):
        sys.stdout.write("\a")
        sys.stdout.flush()
        time.sleep(0.25)


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", UDP_PORT))

print("=" * 46)
print(f" 낙상 감시 시스템 — UDP {UDP_PORT} 대기 중")
print("=" * 46)

last_alarm = 0

while True:
    data, addr = sock.recvfrom(1024)

    if data.decode("utf-8", errors="ignore").strip() != "FALL":
        continue

    # 태그가 신호를 반복 전송하므로 일정 시간 내 중복은 무시한다
    now = time.time()
    if now - last_alarm < DUPLICATE_WINDOW:
        continue
    last_alarm = now

    timestamp = datetime.now().strftime("%H:%M:%S")
    print("\n" + "🚨" * 20)
    print(f"  [{timestamp}] 낙상 감지!  태그 IP: {addr[0]}")
    print("🚨" * 20 + "\n")

    alarm_sound()

    with open("fall_log.txt", "a", encoding="utf-8") as f:
        f.write(f"{datetime.now()} FALL from {addr[0]}\n")