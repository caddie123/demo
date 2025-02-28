#!/usr/bin/env python3
import rospy
import numpy as np
import math
import pyrealsense2 as rs
import torch
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

###############################################
# 기본 파라미터 설정
###############################################

# 카메라 및 YOLO 파라미터
img_width = 640
horizontal_fov_deg = 69.4           # 카메라 수평 FOV (degree)
horizontal_fov_rad = math.radians(horizontal_fov_deg)

desired_distance = 2.0              # 사람 추종 목표 거리 (m)
max_linear_speed = 1.0              # 최대 선속도 (m/s)
max_angular_speed = 1.0             # 최대 회전 속도 (rad/s)

# LiDAR(2D LaserScan) 장애물 회피 파라미터
obstacle_threshold = 2.0            # 장애물 인식 최대 거리 (m)
# 전방 60° 범위: -30° ~ +30° (radian)
scan_fov_min = -math.radians(30)
scan_fov_max = math.radians(30)
# 사람 각도 주변은 장애물로 인식하지 않음 (예: ±10°)
person_ignore_tolerance = math.radians(10)

###############################################
# 전역 변수
###############################################

# 최근 LaserScan 데이터를 저장
latest_scan = None
# 사람이 검출되었을 때, 해당 사람의 각도 (라디안)
person_angle = None

###############################################
# RealSense와 YOLO 초기화
###############################################

# RealSense 파이프라인 설정 (컬러 + 깊이)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# YOLOv5 모델 로드 (pretrained 모델 사용)
rospy.loginfo("Loading YOLOv5 model...")
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.conf = 0.5
PERSON_CLASS_ID = 0

###############################################
# LaserScan 콜백 함수 (2D LiDAR 데이터)
###############################################

def scan_callback(scan_msg):
    global latest_scan
    latest_scan = scan_msg
    # (필요시 디버깅 로그 출력)
    rospy.loginfo_throttle(5, "Received LaserScan with %d points", len(scan_msg.ranges))

###############################################
# 장애물 회피 벡터 계산 함수 (전방 60°만 처리)
###############################################

def compute_avoidance_vector(scan_msg, person_ang):
    """
    LaserScan 데이터를 기반으로 장애물 회피 벡터를 계산합니다.
    - 전방 60° 범위 (scan_fov_min ~ scan_fov_max) 내의 점만 처리.
    - 각 스캔 포인트 중, 측정값이 obstacle_threshold 이하이면,
      (1 - distance/obstacle_threshold) 가중치를 적용해 회피 벡터를 누적합니다.
    - 만약 해당 스캔 각도가 검출된 사람(person_ang) 주변(person_ignore_tolerance)이라면 무시합니다.
    """
    avoidance = np.array([0.0, 0.0])
    if scan_msg is None:
        return avoidance

    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        # 유효한 측정값 확인 및 전방 60° 범위 내인지 확인
        if np.isfinite(r) and r < obstacle_threshold and (angle >= scan_fov_min and angle <= scan_fov_max):
            # 만약 사람이 검출되었고, 이 각도가 사람 각도와 가까우면 무시
            if person_ang is not None and abs(angle - person_ang) < person_ignore_tolerance:
                pass  # 사람은 장애물로 인식하지 않음
            else:
                # 가까울수록 가중치 높음
                weight = 1 - (r / obstacle_threshold)
                # 회피 벡터는 장애물 반대 방향 (즉, -cos(angle), -sin(angle))
                avoidance += np.array([-np.cos(angle), -np.sin(angle)]) * weight
        angle += scan_msg.angle_increment
    return avoidance

###############################################
# 메인 함수
###############################################

def main():
    rospy.init_node("human_follower_2d")
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    rate = rospy.Rate(10)  # 10 Hz
    try:
        while not rospy.is_shutdown():
            # RealSense 프레임 받기
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            # YOLO로 사람 검출
            results = model(color_image)
            detections = results.xyxy[0].cpu().numpy()

            twist = Twist()
            target_vector = np.array([0.0, 0.0])
            global person_angle
            person_angle = None  # 초기화

            person_detected = False
            for *box, conf, cls in detections:
                if int(cls) == PERSON_CLASS_ID:
                    # 첫 번째 사람 검출만 처리
                    x1, y1, x2, y2 = map(int, box)
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    depth_val = depth_frame.get_distance(center_x, center_y)
                    # 카메라 중심 기준으로 각도 계산
                    # (이미지 중심이 0°라 가정, horizontal_fov_deg 분포)
                    angle_deg = (center_x - (img_width / 2)) * (horizontal_fov_deg / img_width)
                    person_angle = math.radians(angle_deg)
                    rospy.loginfo("Person detected: center=(%d,%d), depth=%.2f m, angle=%.1f°",
                                  center_x, center_y, depth_val, angle_deg)
                    # 목표 벡터: 사람과의 거리 오차를 바탕으로, 그 방향으로 이동하도록
                    error = depth_val - desired_distance
                    target_vector = error * np.array([np.cos(person_angle), np.sin(person_angle)])
                    person_detected = True
                    break

            # 장애물 회피 벡터 계산 (전방 60°만 처리)
            avoidance_vector = compute_avoidance_vector(latest_scan, person_angle if person_detected else None)

            # 최종 명령 벡터: 목표 추종 벡터 + 장애물 회피 벡터
            combined_vector = target_vector + avoidance_vector
            norm = np.linalg.norm(combined_vector)
            if norm > 0:
                normalized_vector = combined_vector / norm
            else:
                normalized_vector = np.array([0.0, 0.0])

            # 간단한 속도 매핑: X 성분 → 선속도, Y 성분 → 각속도 (회전 각속도는 arctan2로 계산)
            linear_vel = np.clip(np.dot(normalized_vector, np.array([1, 0])), 0, max_linear_speed)
            angular_vel = np.clip(np.arctan2(normalized_vector[1], normalized_vector[0]),
                                  -max_angular_speed, max_angular_speed)
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel

            rospy.loginfo("Publishing cmd_vel: linear=%.2f, angular=%.2f", linear_vel, angular_vel)
            cmd_pub.publish(twist)
            rate.sleep()

    except Exception as e:
        rospy.logerr("Exception: %s", e)
    finally:
        pipeline.stop()

if __name__ == '__main__':
    main()

