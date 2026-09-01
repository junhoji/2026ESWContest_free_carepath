import cv2
import time
import numpy as np
from ultralytics import YOLO


class VisionProcessor:
    """카메라 프레임에서 추종 대상을 찾고 복도 혼잡도를 판단한다."""

    CAMERA_INDEX = 1
    FRAME_WIDTH = 320
    FRAME_HEIGHT = 240
    CROWD_THRESHOLD = 2
    MIN_BOX_AREA = 1000      # 이보다 작은 검출은 노이즈로 간주
    BLACK_RATIO_MIN = 0.3    # 박스 면적 대비 이 비율 이상 어두우면 대상으로 판정

    def __init__(self, shared_state):
        self.state = shared_state

        self.cam = cv2.VideoCapture(self.CAMERA_INDEX)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)

        # 드라이버가 지난 프레임을 쌓아두지 않도록 버퍼를 최소화
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        print("[비전 스레드] YOLOv8 Nano 신경망 로딩 중... (시간 소요됨)")
        self.model = YOLO('yolov8n.pt')

        self.frame_count = 0

    def process_tracking(self, frame):
        """검출된 사람 중 어두운 옷을 입은 가장 가까운 대상의 위치를 반환한다."""
        results = self.model(frame, classes=[0], verbose=False)
        boxes = results[0].boxes

        if len(boxes) == 0:
            return False, 0.0

        max_area = 0
        target_box = None

        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            area = (x2 - x1) * (y2 - y1)

            if area < self.MIN_BOX_AREA:
                continue

            # 사람 영역만 잘라내 HSV로 변환한 뒤 어두운 픽셀 비율을 구한다
            roi = frame[y1:y2, x1:x2]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 60])
            mask = cv2.inRange(hsv, lower_black, upper_black)

            black_ratio = cv2.countNonZero(mask) / area

            if black_ratio > self.BLACK_RATIO_MIN and area > max_area:
                max_area = area
                target_box = box

        if target_box is None:
            return False, 0.0

        x1, _, x2, _ = target_box.xyxy[0].tolist()
        center_x = (x1 + x2) / 2

        return True, float(center_x)

    def process_crowd(self, frame):
        """검출된 사람 수로 혼잡 여부를 판단한다."""
        results = self.model(frame, classes=[0], verbose=False)
        return len(results[0].boxes) >= self.CROWD_THRESHOLD

    def run(self):
        print("[비전 스레드] 딥러닝 실시간 연산 본격 가동!")

        while True:
            # 드라이버 버퍼에 밀려 있는 지난 프레임을 버리고 최신 것만 사용한다
            for _ in range(3):
                self.cam.grab()

            ret, frame = self.cam.read()

            if not ret:
                print("⚠️ [경고] 카메라 프레임을 읽을 수 없습니다. (선 연결 확인)")
                time.sleep(0.1)
                continue

            self.frame_count += 1

            # 추적과 혼잡도 판단을 프레임마다 번갈아 수행해 연산 부하를 분산한다
            if self.frame_count % 2 == 0:
                target_found, center_x = self.process_tracking(frame)
                with self.state.lock:
                    self.state.target_visible = target_found
                    self.state.target_x = center_x
            else:
                is_crowded = self.process_crowd(frame)
                with self.state.lock:
                    self.state.is_crowded = is_crowded

            # YOLO 추론 시간 자체가 루프 주기를 결정하므로 별도 sleep을 두지 않는다