#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import numpy as np

class PointCloud120DegFilter:
    def __init__(self):
        rospy.init_node("pointcloud_120deg_filter_node", anonymous=True)

        # PointCloud2 데이터 구독
        self.pc_sub = rospy.Subscriber("/unilidar/cloud", PointCloud2, self.pc_callback)

        # 필터링된 PointCloud2 퍼블리시
        self.pc_pub = rospy.Publisher("/filtered_cloud", PointCloud2, queue_size=10)

        # 유지할 각도 범위 (-60° ~ 60°)
        self.min_angle = np.deg2rad(-60)  # -60도 (rad)
        self.max_angle = np.deg2rad(60)   # +60도 (rad)

    def pc_callback(self, msg):
        """
        3D LiDAR(PointCloud2) 데이터를 전방 120도 (-60° ~ 60°) 범위만 필터링
        """
        # 필터링된 포인트 저장 리스트
        filtered_points = []

        # PointCloud2 데이터 읽기
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]

            # 각도 계산 (atan2 사용)
            angle = np.arctan2(y, x)

            # 특정 각도(-60° ~ 60°) 범위 내의 점들만 유지
            if self.min_angle <= angle <= self.max_angle:
                filtered_points.append((x, y, z))

        # 새로운 PointCloud2 메시지 생성
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id  # 기존 LiDAR 프레임 유지

        filtered_cloud = pc2.create_cloud_xyz32(header, filtered_points)

        # 필터링된 포인트 클라우드 퍼블리시
        self.pc_pub.publish(filtered_cloud)

if __name__ == "__main__":
    try:
        PointCloud120DegFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
