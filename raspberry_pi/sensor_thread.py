import time
import socket
import PyLidar3


class SensorProcessor:
    """라이다 거리 측정과 환자 태그의 낙상 신호 수신을 담당한다."""

    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    LIDAR_PORT = "/dev/ttyUSB1"
    FALL_HOLD = 3.0          # 낙상 신호 수신 후 상태를 유지할 시간 (초)

    def __init__(self, shared_state):
        self.state = shared_state

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.setblocking(False)

        self.last_fall_time = 0

        self.lidar = PyLidar3.YdLidarX4(self.LIDAR_PORT)
        self.scan_generator = None

        print("[센서 스레드] 라이다 연결 시도 중...")
        if self.lidar.Connect():
            print("✅ 라이다 연결 성공! 스캔 모터 가동.")
            self.scan_generator = self.lidar.StartScanning()
        else:
            print(f"❌ 라이다 연결 실패. USB 포트({self.LIDAR_PORT})를 확인하세요.")

    def get_averaged_distance(self, lidar_data, target_angle, window=5):
        """지정 각도 주변 측정값을 평균내어 노이즈를 줄인다. 반환 단위는 cm."""
        valid = []
        for angle in range(target_angle - window, target_angle + window + 1):
            dist_mm = lidar_data.get(angle % 360, 0)
            if dist_mm > 0:
                valid.append(dist_mm)

        if valid:
            return sum(valid) / len(valid) / 10.0

        # 유효한 측정값이 없으면 장애물 없음으로 처리
        return 999.0

    def check_fall_signal(self):
        """수신 버퍼를 모두 비우고 낙상 신호가 있었는지 반환한다."""
        got_fall = False
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                if data.decode('utf-8', errors='ignore').strip() == "FALL":
                    got_fall = True
            except BlockingIOError:
                break
        return got_fall

    def run(self):
        print("[센서 스레드] 연산 시작...")

        while True:
            is_fallen = self.check_fall_signal()

            front_dist = 999.0
            right_dist = 999.0

            if self.scan_generator is not None:
                try:
                    lidar_data = next(self.scan_generator)
                    front_dist = self.get_averaged_distance(lidar_data, 0)
                    right_dist = self.get_averaged_distance(lidar_data, 270)
                except Exception as e:
                    print(f"라이다 데이터 읽기 에러: {e}")

            if is_fallen:
                self.last_fall_time = time.time()
                print("🚨 [센서] ESP32 낙상 신호 수신!")

            # 마지막 수신 시점을 기준으로 일정 시간이 지나면 자동 해제된다
            with self.state.lock:
                self.state.front_dist = front_dist
                self.state.right_dist = right_dist
                self.state.fall_detected = (
                    time.time() - self.last_fall_time
                ) < self.FALL_HOLD

            time.sleep(0.05)