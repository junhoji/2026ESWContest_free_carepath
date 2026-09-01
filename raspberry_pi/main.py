import threading
import time
from vision_thread import VisionProcessor
from sensor_thread import SensorProcessor
from motor_thread import MotorController


class RobotState:
    """세 스레드가 공유하는 로봇의 현재 상태. lock으로 접근을 보호한다."""

    def __init__(self):
        self.lock = threading.Lock()

        # 비전 계층
        self.target_x = 0            # 추종 대상의 화면상 x좌표
        self.target_visible = False  # 대상 검출 여부
        self.is_crowded = False      # 복도 혼잡 여부

        # 센서 계층
        self.front_dist = 999.0      # 전방 거리 (cm)
        self.right_dist = 999.0      # 우측 거리 (cm)
        self.fall_detected = False   # 환자 낙상 감지 여부

        # 수동 개입
        self.code_blue = False              # 응급 통행로 확보 모드
        self.code_blue_action_done = False  # 회피 기동 완료 여부


shared_state = RobotState()
motor_system = MotorController()


def execute_wall_following(speed):
    """우측 벽과 일정 거리를 유지하며 주행한다."""
    TARGET_DIST = 30.0
    Kp = 1.2

    with shared_state.lock:
        current_right = shared_state.right_dist

    error = current_right - TARGET_DIST
    motor_system.set_speed(speed, error * Kp)


def emergency_listener():
    """터미널 입력으로 코드블루를 발동하거나 해제한다."""
    while True:
        try:
            cmd = input().strip().lower()
            if cmd == 'c':
                with shared_state.lock:
                    shared_state.code_blue = True
                    shared_state.code_blue_action_done = False
            elif cmd == 'r':
                with shared_state.lock:
                    shared_state.code_blue = False
                    shared_state.code_blue_action_done = False
                print("\n✅ [시스템] 코드블루 해제. 정상 환자 추종 모드로 복귀합니다.\n")
        except Exception:
            pass


def main_controller():
    """우선순위에 따라 로봇의 행동을 결정하는 메인 루프."""
    print("[Main Controller] 지휘 루프 시작")
    print("💡 [팁] 실행 중 터미널에 'c'를 입력하고 엔터를 치면 Code Blue(우측 비켜주기)가 발동됩니다.")
    print("💡 [팁] 복귀하려면 'r'을 입력하고 엔터를 치세요.")

    last_sent_steering = -999.0
    last_sent_speed = -999

    while True:
        with shared_state.lock:
            fall = shared_state.fall_detected
            crowded = shared_state.is_crowded
            front = shared_state.front_dist
            target_seen = shared_state.target_visible
            target_x = shared_state.target_x
            is_code_blue = shared_state.code_blue
            code_blue_done = shared_state.code_blue_action_done

        # 0순위: 응급 의료진 통행로 확보
        if is_code_blue:
            if not code_blue_done:
                RED_BG = "\033[41;97m"
                BLINK = "\033[5m"
                RESET = "\033[0m"
                print(f"{RED_BG}{BLINK}")
                print("==================================================")
                print("           🚨 [ CODE BLUE / 코드 블루 발동 ] 🚨       ")
                print("       응급 의료진 통행을 위해 우측으로 회피 기동합니다.      ")
                print("==================================================")
                print(f"{RESET}")

                SERVO_OFFSET = -35.0
                motor_system.set_speed(70, 90.0 + SERVO_OFFSET)

                # 통행로를 충분히 확보할 때까지 우측으로 이동
                time.sleep(6.0)
                motor_system.stop()

                with shared_state.lock:
                    shared_state.code_blue_action_done = True
                last_sent_speed = 0

            time.sleep(0.05)
            continue

        # 1순위: 환자 낙상
        if fall:
            print("🚨 [상황] 환자 낙상 발생! 즉시 동력 차단!")
            motor_system.stop()
            last_sent_speed = 0

        # 2순위: 복도 혼잡
        elif crowded:
            print("⚠️ [상황] 복도 혼잡. 주행 일시 정지.")
            motor_system.stop()
            last_sent_speed = 0

        # 3순위: 환자 추종
        elif target_seen:
            SERVO_OFFSET = -35.0
            CENTER_X = 187.0
            DEADZONE = 40.0
            Kp = -0.2

            error = target_x - CENTER_X

            if abs(error) <= DEADZONE:
                steering_angle = 0.0
                action = "칼직진"
            else:
                steering_angle = max(min(error * Kp, 35.0), -35.0)
                action = "부드러운 조향"

            final_steering = round(steering_angle + SERVO_OFFSET, 1)

            # 조향각이 1도 이상 변했을 때만 명령을 전송한다
            if abs(final_steering - last_sent_steering) >= 1.0 or last_sent_speed != 70:
                print(f"✅ [상황] {action} (오차:{error:.0f} 조향:{final_steering})")
                motor_system.set_speed(70, final_steering)
                last_sent_steering = final_steering
                last_sent_speed = 70

        # 4순위: 대상 상실
        else:
            motor_system.stop()
            last_sent_speed = 0

        time.sleep(0.05)


if __name__ == "__main__":
    input_thread = threading.Thread(target=emergency_listener, daemon=True)
    input_thread.start()

    vision_processor = VisionProcessor(shared_state)
    sensor_processor = SensorProcessor(shared_state)

    vision_thread = threading.Thread(target=vision_processor.run, daemon=True)
    sensor_thread = threading.Thread(target=sensor_processor.run, daemon=True)

    vision_thread.start()
    sensor_thread.start()

    time.sleep(1)

    try:
        main_controller()
    except KeyboardInterrupt:
        print("\n[비상 정지] 사용자가 시스템을 강제 종료했습니다.")
    finally:
        motor_system.stop()
        try:
            vision_processor.cam.release()
        except Exception:
            pass
        try:
            sensor_processor.lidar.StopScanning()
            sensor_processor.lidar.Disconnect()
        except Exception:
            pass
        time.sleep(0.5)
        print("하드웨어 전력 차단 완료. 시스템 완전히 종료됨.")