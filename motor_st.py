#!/usr/bin/env python3
import rospy
import serial
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class MotorDriver:
    def __init__(self):
        rospy.init_node('motor_driver', anonymous=True)

        # 🚀 MoonWalker 모터 드라이버 시리얼 포트 설정
        try:
            self.arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            rospy.loginfo("✅ MoonWalker 연결 성공: /dev/ttyUSB1")
        except serial.SerialException:
            rospy.logerr("🚨 MoonWalker 연결 실패! 포트 확인 필요 (/dev/ttyUSB1)")
            exit()

        # LiDAR 데이터 구독 (토픽: /min_distance_cloud)
        self.sub = rospy.Subscriber("/min_distance_cloud", PointCloud2, self.lidar_callback)

        # 장애물 감지 범위 (예: 1.2m ~ 1.3m)
        self.OBSTACLE_MIN_DIST = 1.25
        self.OBSTACLE_MAX_DIST = 1.35

        # 명령어 설정
        self.FORWARD_COMMAND = "mvc=12,-12\r\n"    # 전진
        self.TURN_RIGHT_COMMAND = "mvc=22,-5\r\n"   # 오른쪽 회전

        # 모터 상태 및 회피 관련 변수
        self.MOVING = False
        self.avoidance_count = 0          # 회피 횟수
        self.AVOIDANCE_LIMIT = 2          # 2번째 장애물 감지 시 정지

        # 상태 머신 모드: "NORMAL", "EVASION", "IGNORE"
        self.mode = "NORMAL"
        self.evasion_start_time = None  # EVASION 모드 시작 시간
        self.ignore_start_time = None   # IGNORE 모드 시작 시간

        self.EVASION_DURATION = 5.0     # EVASION 모드 지속 시간 (2초)
        self.IGNORE_DURATION = 2.0      # IGNORE 모드 지속 시간 (5초)

        # 최신 LiDAR 최소거리 저장 변수 (NORMAL 상태에서만 업데이트)
        self.last_min_distance = None

        # 타이머: 10Hz 주기로 상태 점검 및 명령 전송
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def lidar_callback(self, msg):
        # NORMAL 상태일 때만 센서 데이터를 업데이트
        if self.mode == "NORMAL":
            min_distance = float('inf')
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point[:3]
                distance = (x**2 + y**2 + z**2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
            self.last_min_distance = min_distance
            rospy.loginfo("📏 최소 거리: {:.2f} m".format(min_distance))

    def timer_callback(self, event):
        current_time = rospy.Time.now()

        # 회피 횟수가 한계에 도달하면 정지
        if self.avoidance_count >= self.AVOIDANCE_LIMIT:
            rospy.logerr("⚠️ {}번째 장애물 감지됨! 로봇 정지.".format(self.AVOIDANCE_LIMIT))
            self.stop_motors()
            return

        if self.mode == "NORMAL":
            # NORMAL 상태: 센서 값에 따라 장애물 감지 판단
            if self.last_min_distance is not None and self.OBSTACLE_MIN_DIST <= self.last_min_distance <= self.OBSTACLE_MAX_DIST:
                rospy.logwarn("🚨 장애물 감지! EVASION 모드 시작")
                self.avoidance_count += 1
                self.mode = "EVASION"
                self.evasion_start_time = current_time
                self.turn_right()  # EVASION 모드 시작 시 한 번만 회전 명령 전송
            else:
                self.move_forward()

        elif self.mode == "EVASION":
            # EVASION 상태: 2초 동안 회전 동작 유지
            elapsed = (current_time - self.evasion_start_time).to_sec()
            if elapsed >= self.EVASION_DURATION:
                rospy.loginfo("✅ EVASION 모드 종료 (2초 경과) → IGNORE 모드 전환, 전진 명령 실행")
                self.mode = "IGNORE"
                self.ignore_start_time = current_time
                self.stop_motors()    # EVASION 모드 종료 시 회전 중지
                self.move_forward()   # 전진 명령 전송
            # EVASION 상태에서는 추가 명령 전송 없이 대기

        elif self.mode == "IGNORE":
            # IGNORE 상태: 5초 동안 센서 값 무시하며 전진
            elapsed = (current_time - self.ignore_start_time).to_sec()
            if elapsed >= self.IGNORE_DURATION:
                rospy.loginfo("✅ IGNORE 모드 종료 (5초 경과) → NORMAL 모드 전환")
                self.mode = "NORMAL"
                self.MOVING = False  # NORMAL 진입 시 MOVING 플래그 리셋
            else:
                self.move_forward()

    def send_command(self, command):
        self.arduino.write(command.encode())
        rospy.loginfo("명령 전송: " + command.strip())

    def move_forward(self):
        if not self.MOVING:
            rospy.loginfo("🚀 전진 명령: FORWARD")
            self.send_command(self.FORWARD_COMMAND)
            self.MOVING = True

    def turn_right(self):
        rospy.logwarn("↪️ 회피 {}회: TURN RIGHT (2초 유지)".format(self.avoidance_count))
        self.send_command(self.TURN_RIGHT_COMMAND)
        self.MOVING = True

    def stop_motors(self):
        if self.MOVING:
            rospy.logwarn("🛑 로봇 정지!")
            self.send_command("mvc=0,0\r\n")
            self.MOVING = False

if __name__ == "__main__":
    try:
        MotorDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
