#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import tf
import pickle
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Imu,Image
import cv2
from cv_bridge import CvBridge

ros_package = rospkg.RosPack()
package_path = ros_package.get_path('xycar_slam_drive')

class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0

        self.yaw = 0.0
        self.v = 0
        self.imu_data = -0.98
        self.state = "path_planning"
        self.stop_line = False

        with open(
                package_path + "/slam_path_new.pkl",
                "rb") as f:
            self.path = pickle.load(f)

        self.rear_x_previous = self.path['x'][0]
        self.rear_y_previous = self.path['y'][0]

        self.ego_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)
        self.ego_imu_sub = rospy.Subscriber("imu", Imu, self.ImuCallBack)

    def ImuCallBack(self, msg):
        self.imu_data = msg.linear_acceleration.z

    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        dx = self.rear_x - self.rear_x_previous
        dy = self.rear_y - self.rear_y_previous
        dist = np.hypot(dx, dy)
        self.v = float(dist) / float(5e-2)
        self.rear_x_previous = self.rear_x
        self.rear_y_previous = self.rear_y

    def GetMsg(self):
        delta = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v,
                               self.path['x'], self.path['y'], self.path['yaw'],
                               0.3, 0.2)

        delta = delta  # *50/30
        if delta < -50:
            delta = -50
        elif delta > 50:
            delta = 50

        msg = xycar_motor()
        if self.imu_data <= -1 and 1.9 < self.rear_y < 2.4:
            self.state = "slope"
        elif self.imu_data <= -2.5:
            self.state = "path_planning"


        if self.state == "slope" and 2.5 < self.rear_y < 2.8 and self.v < 0.009 and 2.4< self.rear_x < 2.8:
           self.state = "path_planning2"


        if self.state == "path_planning":
            msg.angle = delta
            msg.speed = 20
        elif self.state == "path_planning2":
            msg.angle = delta
            msg.speed = 30
        elif self.state == "slope":
            msg.angle = 0
            msg.speed = -100

        return msg

if __name__ == '__main__':
    rospy.init_node("stanley_follower_node")

    motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
    stanley = StanleyController()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        x = stanley.rear_x
        y = stanley.rear_y
        msg = stanley.GetMsg()
        motor_pub.publish(msg)


        r.sleep()
