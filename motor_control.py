#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import serial

# Arduino와 시리얼 통신 설정
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
rospy.init_node('motor_controller')

def callback(msg):
    linear = msg.linear.x  # 전진/후진 속도
    angular = msg.angular.z # 회전 속도

    # Arduino로 보낼 명령 문자열 생성
    command = "{:.2f},{:.2f}\n".format(linear, angular)
    arduino.write(command.encode())

rospy.Subscriber("cmd_vel", Twist, callback)
rospy.spin()
