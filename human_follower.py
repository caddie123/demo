#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import torch
import torch.backends.cudnn as cudnn
from pathlib import Path
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from yolov5_ros.msg import BoundingBox, BoundingBoxes
from robot_ctrl.msg import CenterAndArray, DepthandDeg
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.torch_utils import select_device
from utils.augmentations import letterbox

class DepthCorrection:
    def __init__(self, horizontal_fov=56, min_depth=0.1, max_depth=10.0):
        self.horizontal_fov = np.radians(horizontal_fov)
        self.min_depth = min_depth
        self.max_depth = max_depth 

    def correct_depth(self, depth, pixel_x, image_width):
        angle = self.calculate_angle(pixel_x, image_width)
        correction_factor = self.get_correction_factor(angle)
        corrected_depth = depth * correction_factor
        corrected_depth = np.clip(corrected_depth, self.min_depth, self.max_depth)
        return corrected_depth

    def calculate_angle(self, pixel_x, image_width):
        x_offset_from_center = pixel_x - (image_width / 2)
        angle = (x_offset_from_center / (image_width / 2)) * 28  
        return angle

    def get_correction_factor(self, angle):
        angle_radians = np.radians(angle)
        correction_factor = 1 / np.cos(angle_radians)
        correction_factor = np.clip(correction_factor, 1, 1.2)
        return correction_factor

class HumanFollower:
    def __init__(self):
        rospy.init_node('human_follower', anonymous=True)

        # Parameters
        self.desired_distance = rospy.get_param('~desired_distance', 2.0)
        self.stop_distance = rospy.get_param('~stop_distance', 1.0)
        self.follow_threshold = rospy.get_param('~follow_threshold', 1.5)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.4)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        self.angular_scale = rospy.get_param('~angular_scale', 0.01)

        # YOLOv5 Parameters
        rospack = rospkg.RosPack()
        yolov5_path = rospack.get_path('yolov5_ros') + '/src/yolov5'
        weights = rospy.get_param("~weights", yolov5_path + '/weights/yolov5s.pt')
        self.device = select_device(str(rospy.get_param("~device", "cpu")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn", True))
        self.stride, self.names = self.model.stride, self.model.names
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h", 480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)
        cudnn.benchmark = True
        self.model.warmup(imgsz=(1, 3, *self.img_size))

        self.depth_correction = DepthCorrection(horizontal_fov=56)

        # Subscribers
        self.image_sub = Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = Subscriber('/camera/depth/image_raw', Image)
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.bridge = CvBridge()
        self.person_detected = False
        self.is_following = False

    def sync_callback(self, img_msg, depth_msg):
        detected_persons = self.detect_person(img_msg)
        if not detected_persons:
            rospy.logwarn("No persons detected.")
            self.person_detected = False
            return

        closest_depth = None
        closest_degree = None
        for person in detected_persons:
            xc, yc, x = person
            center_depth, avg_x = self.get_min_depth(depth_msg, x, yc, xc)
            if center_depth is not None and avg_x is not None:
                if closest_depth is None or center_depth < closest_depth:
                    degree = self.depth_correction.calculate_angle(xc, self.img_size[0])
                    corrected_depth = self.depth_correction.correct_depth(center_depth, xc, self.img_size[0])
                    closest_depth = corrected_depth
                    closest_degree = degree

        if closest_depth is not None and closest_degree is not None:
            self.person_detected = True
            self.update_follow_state(closest_depth)
            self.follow_person(closest_depth, closest_degree)
        else:
            rospy.logwarn("No valid depth found for any detected person.")

    def detect_person(self, img_msg):
        try:
            img_cv = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return []

        im, im0 = self.preprocess(img_cv)
        im = torch.from_numpy(im).to(self.device)
        im = im.float() / 255.0
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(pred, 0.75, 0.45, classes=[0], max_det=1000)

        detected_persons = []
        if pred[0] is not None and len(pred[0]):
            pred[0][:, :4] = scale_boxes(im.shape[2:], pred[0][:, :4], im0.shape).round()
            for *xyxy, conf, cls in reversed(pred[0]):
                if self.names[int(cls)] == 'person':
                    xc = int((xyxy[0] + xyxy[2]) / 2)
                    yc = int((xyxy[1] + xyxy[3]) / 2)
                    x = [i for i in range(int(xyxy[0]) + 1, int(xyxy[2]))]
                    detected_persons.append((xc, yc, x))
        return detected_persons

    def preprocess(self, img):
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=True)[0]])
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)
        return img, img0 

    def get_min_depth(self, depth_msg, x, yc, xc):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return None, None

        depth_array = np.array(depth_image, dtype=np.float32)
        if yc >= depth_array.shape[0]:
            return None, None

        valid_depths_and_x = [(depth_array[yc, xi], xi) for xi in x if xi < depth_array.shape[1] and depth_array[yc, xi] > 0]

        if valid_depths_and_x:
            _min_depth, min_x = min(valid_depths_and_x, key=lambda t: t[0])
            avg_x = xc
            min_depth = round(_min_depth / 1000, 1)
            return min_depth, avg_x
        else:
            return None, None

    def update_follow_state(self, distance):
        if distance <= self.stop_distance:
            self.is_following = False
        elif distance >= self.follow_threshold:
            self.is_following = True

    def follow_person(self, distance, angle_deg):
        twist = Twist()
        if self.is_following and distance > self.desired_distance:
            twist.linear.x = min(self.max_linear_speed, 0.5 * (distance - self.desired_distance))
        else:
            twist.linear.x = 0.0

        angle_rad = angle_deg * np.pi / 180
        twist.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angle_rad * self.angular_scale))

        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Published cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.person_detected:
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                rospy.logwarn("No person detected. Robot is stopping.")
            self.person_detected = False
            rate.sleep()

if __name__ == '__main__':
    try:
        follower = HumanFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
