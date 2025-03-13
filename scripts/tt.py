#!/usr/bin/env python3
import rospy
import serial
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import numpy as np
import pyrealsense2 as rs
import torch
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        rospy.init_node("robot_controller", anonymous=True)

        # ★ 모터 드라이버 시리얼 포트 초기화 (MoonWalker)
        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            rospy.loginfo("✅ MoonWalker 연결 성공: /dev/ttyUSB1")
        except serial.SerialException:
            rospy.logerr("🚨 MoonWalker 연결 실패! 포트 확인 필요 (/dev/ttyUSB1)")
            exit()

        # ★ LiDAR 데이터 구독 (PointCloud2)
        rospy.loginfo("📡 LiDAR 데이터 구독 시작 (토픽: /filtered_cloud)")
        rospy.Subscriber("/filtered_cloud", PointCloud2, self.lidar_callback)

        # ★ RealSense 카메라 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # ★ YOLOv5 모델 로드 (사람 탐지)
        rospy.loginfo("Loading YOLOv5 model...")
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.conf = 0.5
        self.PERSON_CLASS_ID = 0

        # ★ 모터 제어 명령 (시리얼로 전송)
        self.FORWARD_COMMAND = "mvc=12,-12\r\n"   # 전진 명령
        self.TURN_RIGHT_COMMAND = "mvc=12,-5\r\n"  # 오른쪽 회전 명령
        self.STOP_COMMAND = "mvc=0,0\r\n"           # 정지 명령

        # ★ 모터 상태 변수
        self.MOVING = False
        self.OBSTACLE_DETECTED = False
        self.avoidance_mode = False
        self.avoidance_start_time = None
        self.avoidance_duration = 0.7  # 회피 명령 지속 시간 (초)

        # ★ 사람 추종 및 거리 유지 파라미터 (카메라)
        self.desired_distance = 2.0  # 목표 거리 (m)
        self.stop_distance = 1.0     # 정지할 거리 (m)
        self.last_seen_direction = 0 # 마지막으로 본 사람의 방향 (-1: 왼쪽, 1: 오른쪽)

        # ★ LiDAR 필터링 파라미터 (장애물 감지)
        self.obstacle_min_distance = 1.3  # 장애물 감지 최소 거리 (m)
        self.obstacle_max_distance = 1.5  # 장애물 감지 최대 거리 (m)
        self.min_z = 0.0                # 높이 하한 (예: 바닥)
        self.max_z = 1.0                # 높이 상한 (예: 1m)
        self.prev_min_distance = self.obstacle_max_distance

        self.rate = rospy.Rate(10)
        self.run()

    def lidar_callback(self, msg):
        """
        LiDAR 데이터를 처리하여 전방 120° (-60° ~ +60°) 내에서 
        1.3m ~ 1.5m 범위의 장애물만 필터링합니다.
        """
        filtered_points = []
        min_distance_found = float('inf')
        count = 0

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]
            # 높이 필터링: 바닥과 너무 높은 점은 무시
            if self.min_z <= z <= self.max_z:
                # 각도 계산 (도 단위)
                angle_deg = np.degrees(np.arctan2(y, x))
                # 전방 120° (-60° ~ +60°) 범위 내의 점만 사용
                if -60 <= angle_deg <= 60:
                    distance = np.sqrt(x**2 + y**2 + z**2)
                    # 지정된 거리 범위 내의 장애물만 고려 (1.3m ~ 1.5m)
                    if self.obstacle_min_distance <= distance <= self.obstacle_max_distance:
                        filtered_points.append((x, y, z))
                        count += 1
                        min_distance_found = min(min_distance_found, distance)

        if min_distance_found < float('inf'):
            self.prev_min_distance = min_distance_found
            self.OBSTACLE_DETECTED = True
        else:
            min_distance_found = self.prev_min_distance
            self.OBSTACLE_DETECTED = False

        rospy.loginfo(f"📏 LiDAR: 최소 장애물 거리 = {min_distance_found:.2f} m (감지 여부: {self.OBSTACLE_DETECTED}), 포인트 수 = {count}")

    def get_camera_data(self):
        """
        RealSense 카메라에서 컬러와 깊이 프레임을 가져옵니다.
        """
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        color_image = np.asanyarray(color_frame.get_data())
        return color_image, depth_frame

    def detect_person(self, image):
        """
        YOLOv5를 이용해 사람을 탐지하고, bounding box의 중앙 좌표를 반환합니다.
        """
        results = self.model(image)
        detections = results.xyxy[0].cpu().numpy()
        for *box, conf, cls in detections:
            if int(cls) == self.PERSON_CLASS_ID:
                x1, y1, x2, y2 = map(int, box)
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                return center_x, center_y
        return None, None

    # ★ 모터 제어 함수들 (시리얼 통신으로 명령 전송)
    def stop_motors(self):
        if self.MOVING:
            rospy.logwarn("🛑 로봇 정지!")
            self.arduino.write(self.STOP_COMMAND.encode())
            self.MOVING = False
        self.OBSTACLE_DETECTED = True

    def move_forward(self):
        if not self.MOVING and not self.OBSTACLE_DETECTED:
            rospy.loginfo("🚀 전진 명령 전송")
            self.arduino.write(self.FORWARD_COMMAND.encode())
            self.MOVING = True
            self.avoidance_mode = False

    def turn_right(self):
        rospy.loginfo("↪️ 오른쪽 회전 명령 전송")
        self.arduino.write(self.TURN_RIGHT_COMMAND.encode())
        self.MOVING = True

    def run(self):
        """
        메인 실행 루프:
          1. LiDAR를 통한 장애물 감지 시, 회피 모드로 전환하여 오른쪽 회전 명령을 시리얼로 전송합니다.
          2. 장애물이 없으면 카메라로 사람을 탐지하여 사람과의 거리에 따라 전진 또는 정지 명령을 보냅니다.
        """
        while not rospy.is_shutdown():
            # ★ 장애물 감지 (LiDAR 기반)
            if self.OBSTACLE_DETECTED:
                if not self.avoidance_mode:
                    rospy.logwarn("🚨 장애물 감지됨! 회피 시작")
                    self.avoidance_mode = True
                    self.OBSTACLE_DETECTED = True
                    self.avoidance_start_time = rospy.Time.now()
                    self.turn_right()
                else:
                    if (rospy.Time.now() - self.avoidance_start_time).to_sec() < self.avoidance_duration:
                        self.turn_right()
                    else:
                        rospy.loginfo("✅ 회피 완료, 전진 재개")
                        self.OBSTACLE_DETECTED = False
                        self.avoidance_mode = False
                        self.stop_motors()  # 정지 후 전진 재개
                        self.move_forward()
                self.rate.sleep()
                continue

            # ★ 카메라를 통한 사람 탐지 및 추종
            color_image, depth_frame = self.get_camera_data()
            if color_image is None:
                self.rate.sleep()
                continue

            center_x, center_y = self.detect_person(color_image)
            if center_x is not None:
                depth_value = depth_frame.get_distance(center_x, center_y)
                angle_deg = (center_x - (640/2)) * (69.4/640)
                rospy.loginfo(f"👤 사람 감지: Depth = {depth_value:.2f} m, Angle = {angle_deg:.1f}°")
                self.last_seen_direction = -1 if center_x < 320 else 1

                # ★ 사람과의 거리 기반 모터 제어
                if depth_value > self.desired_distance:
                    self.move_forward()
                elif depth_value > self.stop_distance:
                    self.move_forward()  # 일정 거리를 유지하기 위해 전진 명령 유지
                else:
                    self.stop_motors()
            else:
                rospy.logwarn("🔍 사람을 찾지 못함, 검색 중...")
                # 마지막으로 본 방향으로 회전하며 재탐색 (여기서는 오른쪽 회전)
                self.turn_right()

            self.rate.sleep()

if __name__ == "__main__":
    try:
        RobotController()
    except rospy.ROSInterruptException:
        pass
