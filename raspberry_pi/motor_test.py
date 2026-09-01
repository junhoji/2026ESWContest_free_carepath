"""구동부 단독 검증 스크립트. 바퀴를 지면에서 띄운 상태로 실행할 것."""
import time
from motor_thread import MotorController

motor = MotorController()
print("=== 모터 테스트 시작 ===")
print("⚠️ 바퀴를 공중에 띄운 상태로 하세요\n")

try:
    # 조향 동작 확인
    print("[1] 조향 중앙")
    motor.set_speed(0, 0)
    time.sleep(1.5)

    print("[2] 조향 우측 25도")
    motor.set_speed(0, 25)
    time.sleep(1.5)

    print("[3] 조향 중앙")
    motor.set_speed(0, 0)
    time.sleep(1.5)

    print("[4] 조향 좌측 25도")
    motor.set_speed(0, -25)
    time.sleep(1.5)

    print("[5] 조향 중앙 복귀")
    motor.set_speed(0, 0)
    time.sleep(1.5)

    # 구동 동작 확인
    print("\n[6] 전진 속도 40")
    motor.set_speed(40, 0)
    time.sleep(2)

    print("[7] 정지")
    motor.stop()
    time.sleep(1.5)

    print("[8] 후진 속도 -40")
    motor.set_speed(-40, 0)
    time.sleep(2)

    print("[9] 정지")
    motor.stop()
    time.sleep(1.5)

    # 주행 중 조향 확인
    print("\n[10] 전진하며 우회전")
    motor.set_speed(40, 25)
    time.sleep(2)

    print("[11] 전진하며 좌회전")
    motor.set_speed(40, -25)
    time.sleep(2)

    print("\n=== 테스트 완료 ===")

except KeyboardInterrupt:
    print("\n[중단]")
finally:
    motor.stop()
    time.sleep(0.3)
    print("모터 정지 완료")