from robot_hat import Motor, PWM, Pin, Servo, utils


class MotorController:
    """애커먼 조향 구동부. 뒷바퀴로 추진하고 앞바퀴 서보로 방향을 잡는다."""

    STEER_OFFSET = -15    # 조향 기구의 기계적 영점 오차 보정값
    MAX_ANGLE = 45        # 조향 명령 허용 범위
    SERVO_LIMIT = 60      # 서보 물리 가동 범위
    MAX_SPEED = 100

    LEFT_CALIB = -1.0     # 좌측 모터 배선 방향 보정
    RIGHT_CALIB = 1.0

    def __init__(self):
        utils.reset_mcu()

        # M1 포트가 좌측 뒷바퀴, M2 포트가 우측 뒷바퀴
        self.left_motor = Motor(PWM("P13"), Pin("D4"))
        self.right_motor = Motor(PWM("P12"), Pin("D5"))
        self.steer_servo = Servo("P2")

        self.steer_servo.angle(self.STEER_OFFSET)
        self.stop()

    def set_speed(self, speed, steering=0):
        """조향각과 주행 속도를 동시에 지정한다."""
        # 명령값을 먼저 제한한 뒤 영점 보정을 더해야 좌우 가동 범위가 대칭이 된다
        steering = max(min(steering, self.MAX_ANGLE), -self.MAX_ANGLE)
        servo_angle = max(min(steering + self.STEER_OFFSET,
                              self.SERVO_LIMIT), -self.SERVO_LIMIT)
        self.steer_servo.angle(servo_angle)

        left_speed = max(min(speed * self.LEFT_CALIB, self.MAX_SPEED), -self.MAX_SPEED)
        right_speed = max(min(speed * self.RIGHT_CALIB, self.MAX_SPEED), -self.MAX_SPEED)

        self.left_motor.speed(left_speed)
        self.right_motor.speed(right_speed)

    def stop(self):
        """구동을 차단하고 조향을 중립으로 되돌린다."""
        self.left_motor.speed(0)
        self.right_motor.speed(0)
        self.steer_servo.angle(self.STEER_OFFSET)