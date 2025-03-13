#!/usr/bin/env python3
import rospy
import pyrealsense2 as rs
import numpy as np
import torch
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("human_follower", anonymous=True)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # RealSense 설정
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)

    # YOLOv5 모델 로드
    rospy.loginfo("Loading YOLOv5 model...")
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    model.conf = 0.5
    PERSON_CLASS_ID = 0

    # 파라미터 설정
    img_width = 640
    horizontal_fov = 69.4  # (degree)
    desired_distance = 2.0     # 목표 거리 (m)
    stop_distance = 1.0        # 정지할 거리 (m)
    max_linear_speed = 0.3     # 최대 속도 (m/s)
    max_angular_speed = 0.5    # 최대 회전 속도 (rad/s)
    angular_scale = 0.2       # 회전 속도 스케일

    # 업데이트 주기를 20Hz로 설정하여 더 빠른 반응 제공
    rate = rospy.Rate(20)

    # 스무딩(Low-Pass Filtering)을 위한 변수 (alpha를 높여 현재값에 더 많은 가중치 부여)
    prev_linear = 0.0
    prev_angular = 0.0
    alpha = 0.8  # 0.0~1.0, 1.0이면 필터링 없이 바로 반영됨

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            results = model(color_image)
            detections = results.xyxy[0].cpu().numpy()

            twist = Twist()
            person_detected = False

            for *box, conf, cls in detections:
                if int(cls) == PERSON_CLASS_ID:
                    x1, y1, x2, y2 = map(int, box)
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    depth_value = depth_frame.get_distance(center_x, center_y)
                    angle_deg = (center_x - (img_width / 2)) * (horizontal_fov / img_width)

                    rospy.loginfo("Person detected: Center=(%d, %d), Depth=%.2fm, Angle=%.1f°",
                                  center_x, center_y, depth_value, angle_deg)

                    if depth_value > desired_distance:
                        twist.linear.x = max_linear_speed
                    elif stop_distance < depth_value <= desired_distance:
                        twist.linear.x = max_linear_speed * ((depth_value - stop_distance) / (desired_distance - stop_distance))
                    else:
                        twist.linear.x = 0.0

                    angle_rad = angle_deg * np.pi / 180.0
                    twist.angular.z = max(-max_angular_speed, min(max_angular_speed, angle_rad * angular_scale))
                    person_detected = True
                    break

            if not person_detected:
                rospy.logwarn("No person detected. Robot is stopping.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # 스무딩 적용 (alpha=0.8로 현재 명령을 빠르게 반영)
            smoothed_linear = alpha * twist.linear.x + (1 - alpha) * prev_linear
            smoothed_angular = alpha * twist.angular.z + (1 - alpha) * prev_angular
            prev_linear = smoothed_linear
            prev_angular = smoothed_angular

            twist.linear.x = smoothed_linear
            twist.angular.z = smoothed_angular

            cmd_pub.publish(twist)
            rate.sleep()

    except Exception as e:
        rospy.logerr("Exception: %s", e)
    finally:
        pipeline.stop()

if __name__ == '__main__':
    main()
