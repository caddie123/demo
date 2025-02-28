#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import numpy as np

class LidarTest:
    def __init__(self):
        rospy.init_node("lidar_test_node", anonymous=True)

        # PointCloud2 데이터 구독 (토픽 변경: /filtered_cloud)
        rospy.loginfo("📡 LiDAR 데이터 구독 시작 (토픽: /filtered_cloud)")
        self.pc_sub = rospy.Subscriber("/filtered_cloud", PointCloud2, self.pc_callback)

        # 필터링된 PointCloud2 퍼블리시
        self.pc_pub = rospy.Publisher("/min_distance_cloud", PointCloud2, queue_size=10)

        # 유지할 높이 범위 (0m ~ 1m)
        self.min_z = 0.0  # 바닥 (0m)
        self.max_z = 1.0  # 1m 높이까지

        # 최소 및 최대 거리 필터
        self.min_distance = 0.3  # 최소 0.3m 이상
        self.max_distance = 1.5  # 최대 1.5m 이하

        # 업데이트 주파수 설정 (10Hz)
        self.rate = rospy.Rate(10)  # 1초에 10번 실행

        # 이전 최소 거리값 (초기값: 최대 거리)
        self.prev_min_distance = self.max_distance  

    def pc_callback(self, msg):
        """
        3D LiDAR(PointCloud2) 데이터를 Z 범위(0m ~ 1m) 내에서 필터링하고,
        최소 거리(0.3m 이상, 최대 2.0m 이하) 값을 출력
        """
        rospy.loginfo("📡 LiDAR 데이터 수신됨!")
        filtered_points = []
        min_distance_found = float('inf')
        count = 0

        # PointCloud2 데이터 읽기
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]
            if self.min_z <= z <= self.max_z:
                filtered_points.append((x, y, z))
                count += 1
                distance = np.sqrt(x**2 + y**2 + z**2)
                # 0.3m 이상, 2.0m 이하인 점들만 고려
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
        filtered_cloud = pc2.create_cloud_xyz32(header, filtered_points)
        self.pc_pub.publish(filtered_cloud)
        self.rate.sleep()


if __name__ == "__main__":
    try:
        LidarTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
