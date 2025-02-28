#!/usr/bin/env python3
import rospy
import serial
import re
import threading
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# 🚀 MoonWalker 모터 드라이버 시리얼 포트 설정
try:
    arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
    rospy.loginfo("✅ MoonWalker 연결 성공: /dev/ttyUSB1")
except serial.SerialException:
    rospy.logerr("🚨 MoonWalker 연결 실패! 포트 확인 필요 (/dev/ttyUSB1)")
    exit()

# 🎯 ROS 노드 초기화
rospy.init_node('motor_driver')

# 🚧 장애물 감지 거리(원래 0.5~0.6m 설정)
OBSTACLE_MIN_DIST = 0.5
OBSTACLE_MAX_DIST = 0.6

# → 하지만 hysteresis를 위해 새 임계값 정의
STOP_THRESHOLD = 0.65   # 이 값 이하이면 모터 정지
START_THRESHOLD = 0.95  # 이 값 이상이면 모터 재시작

# 🚀 기본 이동 속도 (0.3m/s)
DEFAULT_SPEED = 0.3
MOVING = False             # 현재 로봇이 움직이고 있는지 여부
OBSTACLE_DETECTED = False  # 장애물 감지 상태

# 🛑 모터 정지 함수: 모터에 "mvc=0,0" 전송
def stop_motors():
    global MOVING, OBSTACLE_DETECTED
    if MOVING:
        command = "mvc=0,0\r\n"
        rospy.logwarn("🛑 로봇 정지!")
        arduino.write(command.encode())
        MOVING = False
        OBSTACLE_DETECTED = True

# 🚀 모터 주행 함수: 0.3m/s로 이동 (오른쪽 바퀴는 양수, 왼쪽 바퀴는 음수)
def move_forward():
    global MOVING, OBSTACLE_DETECTED
    if not MOVING and not OBSTACLE_DETECTED:
        WHEEL_RADIUS = 0.135  # m
        CONVERSION_FACTOR = 60.0 / (2 * 3.141592653589793 * WHEEL_RADIUS)
        speed = int(DEFAULT_SPEED * CONVERSION_FACTOR)
        command = f"mvc={speed},{-speed}\r\n"
        rospy.loginfo(f"🚀 이동 시작: {command.strip()}")
        arduino.write(command.encode())
        MOVING = True

# 🚧 장애물 감지 콜백 함수: /min_distance_cloud 토픽의 데이터를 이용하여 모터 제어
def callback_obstacle(msg):
    global MOVING, OBSTACLE_DETECTED
    min_distance = float('inf')
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point[:3]
        distance = (x**2 + y**2 + z**2) ** 0.5
        if distance < min_distance:
            min_distance = distance

    rospy.loginfo(f"📏 최소 거리: {min_distance:.2f} m")
    
    # hysteresis 적용: 
    if not OBSTACLE_DETECTED and min_distance < STOP_THRESHOLD:
        rospy.logwarn(f"🚨 장애물 감지됨! (거리: {min_distance:.2f}m) → 모터 정지")
        stop_motors()
    elif OBSTACLE_DETECTED and min_distance > START_THRESHOLD:
        rospy.loginfo(f"✅ 장애물 사라짐! (거리: {min_distance:.2f}m) → 모터 재시작")
        OBSTACLE_DETECTED = False
        move_forward()
    else:
        # 장애물이 없는 경우에도 계속 주행 유지
        if not OBSTACLE_DETECTED:
            move_forward()

# 🛠 ROS 구독 설정: /min_distance_cloud 토픽 구독
rospy.Subscriber("/min_distance_cloud", PointCloud2, callback_obstacle)

# 🛠 ROS 메인 루프 실행 (10Hz)
rate = rospy.Rate(10)
try:
    move_forward()  # 프로그램 시작 시 0.3m/s 이동 시작
    while not rospy.is_shutdown():
        rate.sleep()
except KeyboardInterrupt:
    rospy.logwarn("🛑 프로그램 종료! 로봇 정지 명령 전송 중...")
    stop_motors()
    rospy.sleep(1)
