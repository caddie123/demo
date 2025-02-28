#!/usr/bin/env python3
import rospy
import serial
import threading
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

# 🚀 MoonWalker 모터 드라이버 시리얼 포트 설정
try:
    arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
    rospy.loginfo("✅ MoonWalker 연결 성공: /dev/ttyUSB1")
except serial.SerialException:
    rospy.logerr("🚨 MoonWalker 연결 실패! 포트 확인 필요 (/dev/ttyUSB1)")
    exit()

# 🎯 ROS 노드 초기화
rospy.init_node('motor_driver')

# 🚧 장애물 감지 임계값 (회피 시작: 1.0m ~ 1.2m)
OBSTACLE_MIN_DIST = 1.3
OBSTACLE_MAX_DIST = 1.5

# 🚀 기본 이동 속도 (0.3m/s)
DEFAULT_SPEED = 0.3
# 전진 명령: (오른쪽 바퀴 +, 왼쪽 바퀴 -) → "mvc=21,-21" (예)
FORWARD_COMMAND = "mvc=21,-21\r\n"
# 회피(오른쪽 회전) 명령: (오른쪽 바퀴 +, 왼쪽 바퀴 덜 역회전) → "mvc=21,-10\r\n"
TURN_RIGHT_COMMAND = "mvc=21,-10\r\n"

MOVING = False
OBSTACLE_DETECTED = False
avoidance_mode = False
avoidance_start_time = None
avoidance_duration = 0.7  # 회피 명령 지속 시간 (초)

# 🛑 모터 정지 함수
def stop_motors():
    global MOVING, OBSTACLE_DETECTED
    if MOVING:
        command = "mvc=0,0\r\n"
        rospy.logwarn("🛑 로봇 정지!")
        arduino.write(command.encode())
        MOVING = False
    OBSTACLE_DETECTED = True

# 🚀 전진 명령 함수
def move_forward():
    global MOVING, OBSTACLE_DETECTED, avoidance_mode
    if not MOVING and not OBSTACLE_DETECTED:
        rospy.loginfo(f"🚀 이동 시작: {FORWARD_COMMAND.strip()}")
        arduino.write(FORWARD_COMMAND.encode())
        MOVING = True
        avoidance_mode = False

# ↪️ 회피 명령 함수 (오른쪽으로 커브)
def turn_right():
    global MOVING
    rospy.loginfo(f"↪️ 회피: 오른쪽 회전 명령: {TURN_RIGHT_COMMAND.strip()}")
    arduino.write(TURN_RIGHT_COMMAND.encode())
    MOVING = True

# 🚧 장애물 감지 콜백 함수: /min_distance_cloud 토픽의 데이터를 이용하여 모터 제어
def callback_obstacle(msg):
    global MOVING, OBSTACLE_DETECTED, avoidance_mode, avoidance_start_time
    min_distance = float('inf')
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point[:3]
        distance = (x**2 + y**2 + z**2) ** 0.5
        if distance < min_distance:
            min_distance = distance

    rospy.loginfo(f"📏 최소 거리: {min_distance:.2f} m")
    
    # 회피 모드 진입 조건: 최소 거리가 1.0~1.2m 사이이면 회피 명령 실행
    if OBSTACLE_MIN_DIST <= min_distance <= OBSTACLE_MAX_DIST:
        if not avoidance_mode:
            rospy.logwarn(f"🚨 장애물 감지됨! (거리: {min_distance:.2f}m) → 오른쪽 회전 시작")
            avoidance_mode = True
            OBSTACLE_DETECTED = True
            avoidance_start_time = rospy.Time.now()
            turn_right()
        else:
            if (rospy.Time.now() - avoidance_start_time).to_sec() < avoidance_duration:
                turn_right()
            else:
                rospy.loginfo(f"✅ 회피 완료 → 직진 재시작 (거리: {min_distance:.2f}m)")
                OBSTACLE_DETECTED = False
                avoidance_mode = False
                stop_motors()  # 정지 후 전진 재개
                move_forward()
    else:
        if not avoidance_mode:
            move_forward()

# 🛠 ROS 구독 설정
rospy.Subscriber("/min_distance_cloud", PointCloud2, callback_obstacle)

# 🛠 ROS 메인 루프 실행 (10Hz)
rate = rospy.Rate(10)
try:
    move_forward()  # 프로그램 시작 시 전진
    while not rospy.is_shutdown():
        rate.sleep()
except KeyboardInterrupt:
    rospy.logwarn("🛑 프로그램 종료! 로봇 정지 명령 전송 중...")
    stop_motors()
    rospy.sleep(1)
