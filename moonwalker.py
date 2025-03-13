#!/usr/bin/env python3
import rospy
import serial
import re
from geometry_msgs.msg import Twist, PointStamped

# 🚀 MoonWalker 모터 드라이버 시리얼 포트 설정
try:
    arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    rospy.loginfo("✅ MoonWalker 연결 성공: /dev/ttyUSB0")
except serial.SerialException:
    rospy.logerr("🚨 MoonWalker 연결 실패! 포트 확인 필요 (/dev/ttyUSB0)")
    exit()

# 🎯 ROS 노드 초기화
rospy.init_node('moonwalker_controller')

# 🎯 엔코더 데이터 퍼블리셔
encoder_pub = rospy.Publisher('/encoder_data', PointStamped, queue_size=10)

# 🎯 `cmd_vel` 마지막 수신 시간 (자동 정지 기능)
last_cmd_time = rospy.Time.now()

# 🎯 이전 엔코더 값 (변화 감지용)
prev_left_enc = None
prev_right_enc = None

# 🛑 **모터 정지 함수**
def stop_motors():
    command = "mvc=0,0\r\n"
    rospy.loginfo("🛑 STOP: Sending to MoonWalker: " + command)
    arduino.write(command.encode())

# 🎯 **cmd_vel 콜백 함수 (회전 속도 보정 포함)**
def callback(msg):
    global last_cmd_time
    last_cmd_time = rospy.Time.now()  # 최신 명령 시간 업데이트

    linear = msg.linear.x  # m/s
    angular = msg.angular.z  # rad/s

    # 🛑 **속도가 0이면 자동 정지 명령 전송**
    if abs(linear) < 0.01 and abs(angular) < 0.01:
        stop_motors()
        return

    # ⚙️ **바퀴 기하학적 상수**
    WHEEL_RADIUS = 0.135  # m (270mm / 2)
    AXLE_LENGTH = 0.904   # m (바퀴 간 거리)
    CONVERSION_FACTOR = 60.0 / (2 * 3.141592653589793 * WHEEL_RADIUS)

    angular_scale = 2.0  # 회전 속도 보정 계수 (필요시 조정)

    left_speed = -(linear + (angular * AXLE_LENGTH / 2.0) * angular_scale) * CONVERSION_FACTOR
    right_speed = (linear - (angular * AXLE_LENGTH / 2.0) * angular_scale) * CONVERSION_FACTOR

    # 🛠 MoonWalker에 속도 명령 전송
    command = f"mvc={int(right_speed)},{int(left_speed)}\r\n"
    rospy.loginfo(f"🚀 Sending to MoonWalker: {command.strip()}")
    arduino.write(command.encode())

# 🎯 **MoonWalker 엔코더 데이터를 가져와 퍼블리시하는 함수**
def publish_encoder():
    global prev_left_enc, prev_right_enc
    arduino.write("mp\r\n".encode())  # 엔코더 데이터 요청
    rospy.sleep(0.1)  # 응답 대기

    response = arduino.readline().decode().strip()
    if "=" not in response or "," not in response:
        rospy.logwarn(f"⚠️ 잘못된 응답 무시: {response}")
        return  

    try:
        _, values = response.split("=")  # "mp=-12345,67890" → "-12345,67890"
        left_enc, right_enc = values.split(",")

        # 🎯 `re.sub`을 사용해서 숫자(- 포함) 이외의 문자 제거
        left_enc = re.sub(r"[^\d.-]", "", left_enc)
        right_enc = re.sub(r"[^\d.-]", "", right_enc)

        # 🛠 **이전 값과 비교해서 변화가 없으면 퍼블리시하지 않음**
        if prev_left_enc == left_enc and prev_right_enc == right_enc:
            return  

        prev_left_enc = left_enc
        prev_right_enc = right_enc

        encoder_msg = PointStamped()
        encoder_msg.header.stamp = rospy.Time.now()
        encoder_msg.point.x = float(left_enc)
        encoder_msg.point.y = float(right_enc)
        encoder_msg.point.z = 0.0  # 필요 시 추가 데이터

        encoder_pub.publish(encoder_msg)
        rospy.loginfo(f"📊 Encoder Data: Left={left_enc}, Right={right_enc}")

    except ValueError as e:
        rospy.logerr(f"🚨 엔코더 데이터 변환 오류: {response} → {e}")

# 🎯 **일정 시간 동안 `cmd_vel`이 없으면 자동 정지**
def check_stop():
    global last_cmd_time
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if (rospy.Time.now() - last_cmd_time).to_sec() > 1.0:  # 1초 동안 cmd_vel 없으면 정지
            stop_motors()
        rate.sleep()

# 🛠 **ROS 구독 설정**
rospy.Subscriber("cmd_vel", Twist, callback)

# 🛠 **엔코더 퍼블리싱 쓰레드 실행**
import threading
stop_thread = threading.Thread(target=check_stop)
stop_thread.daemon = True
stop_thread.start()

# 🛠 **ROS 메인 루프 실행**
rate = rospy.Rate(10)
try:
    while not rospy.is_shutdown():
        publish_encoder()
        rate.sleep()
except KeyboardInterrupt:
    rospy.logwarn("🛑 프로그램 종료! 로봇 정지 명령 전송 중...")
    stop_motors()  # 🚨 MoonWalker 정지 명령 실행
    rospy.sleep(1)  # 명령 적용될 시간 확보
