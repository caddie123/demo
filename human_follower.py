#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from robot_ctrl.msg import DepthandDeg
from std_msgs.msg import Bool

class HumanFollower:
    def __init__(self):
        rospy.init_node('human_follower', anonymous=True)

        # Parameters
        self.desired_distance = rospy.get_param('~desired_distance', 2.0)  # 원하는 거리 (2미터)
        self.stop_distance = rospy.get_param('~stop_distance', 1.0)  # 멈출 거리 (1미터 이내)
        self.follow_threshold = rospy.get_param('~follow_threshold', 1.5)  # 다시 추종 시작 거리 (1.5미터)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.4)  # 최대 선형 속도 (m/s)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)  # 최대 각속도 (rad/s)
        self.angular_scale = rospy.get_param('~angular_scale', 0.01)  # 각도 조절 스케일

        # Subscribers
        self.depth_and_deg_sub = rospy.Subscriber('/depth_and_deg', DepthandDeg, self.depth_and_deg_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Tracking state
        self.person_detected = False
        self.is_following = False

    def depth_and_deg_callback(self, msg):
        # 사람 인식 시 거리와 각도 정보 수신
        center_depth = msg.center_depth  # 사람까지의 거리 (m)
        degree = msg.deg  # 사람의 각도 (deg)

        self.person_detected = True
        self.update_follow_state(center_depth)
        self.follow_person(center_depth, degree)

    def update_follow_state(self, distance):
        if distance <= self.stop_distance:
            self.is_following = False  # 사람이 너무 가까우면 멈춤
        elif distance >= self.follow_threshold:
            self.is_following = True  # 사람이 멀어지면 추종 시작

    def follow_person(self, distance, angle_deg):
        twist = Twist()

        if self.is_following and distance > self.desired_distance:
            # 거리 기반 선형 속도 제어
            twist.linear.x = min(self.max_linear_speed, 0.5 * (distance - self.desired_distance))
        else:
            twist.linear.x = 0.0  # 멈춤 조건

        # 각도 기반 회전 속도 제어
        angle_rad = angle_deg * 3.14159 / 180  # 각도를 라디안으로 변환
        twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angle_rad * self.angular_scale))

        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Published cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.person_detected:
                # 사람을 인식하지 못한 경우 정지 상태 유지
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                rospy.logwarn("No person detected. Robot is stopping.")
            self.person_detected = False  # 매 사이클마다 리셋
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = HumanFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
