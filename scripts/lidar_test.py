#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import numpy as np

class LidarTest:
    def __init__(self):
        rospy.init_node("lidar_test_node", anonymous=True)

        rospy.loginfo("📡 LiDAR 데이터 구독 시작 (토픽: /filtered_cloud)")
        self.pc_sub = rospy.Subscriber("/filtered_cloud", PointCloud2, self.pc_callback)

        self.pc_pub = rospy.Publisher("/min_distance_cloud", PointCloud2, queue_size=10)

        # 유지할 높이 범위 (0m ~ 1m)
        self.min_z = 0.0
        self.max_z = 1.0

        # 최소 및 최대 거리 필터
        self.min_distance = 0.3
        self.max_distance = 1.5

        # 이전 최소 거리값 (초기값: 최대 거리)
        self.prev_min_distance = self.max_distance  

        # 처리된 결과 저장 변수 (publish 대상)
        self.filtered_cloud = None

        # Timer 설정: 10Hz로 주기적 publish
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def pc_callback(self, msg):
        rospy.loginfo("📡 LiDAR 데이터 수신됨!")
        filtered_points = []
        min_distance_found = float('inf')
        count = 0

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]
            if self.min_z <= z <= self.max_z:
                filtered_points.append((x, y, z))
                count += 1
                distance = np.sqrt(x**2 + y**2 + z**2)
                if self.min_distance <= distance <= 2.0:
                    min_distance_found = min(min_distance_found, distance)

        if min_distance_found < float('inf'):
            self.prev_min_distance = min_distance_found
        else:
            min_distance_found = self.prev_min_distance

        rospy.loginfo(f"📏 최소 거리: {min_distance_found:.2f} m")
        rospy.loginfo(f"🔎 필터링된 포인트 개수: {count}")

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id
        self.filtered_cloud = pc2.create_cloud_xyz32(header, filtered_points)

    def timer_callback(self, event):
        if self.filtered_cloud is not None:
            self.pc_pub.publish(self.filtered_cloud)

if __name__ == "__main__":
    try:
        LidarTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
