#!/usr/bin/env python
import os
import socket

import rospy

import mr_ros_pb2
from usv_msgs.msg import SpeedCourse
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import tf
from math import fmod, pi

class mr_obs_connector:

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        ip = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 10010)
        self.host = (ip, port)

        self.true_heading = 0

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('speed_course', SpeedCourse, self.handleSpeedCourseMsg)
        rospy.Subscriber('imu/data', Imu, self.handleImuMsg)


    def handleSpeedCourseMsg(self, msg):
        
        # Get heading from TF
        try:
            (trans,rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('No transform from map to base_link. Assuming they are the same...')
            rot = [0, 0, 0, 1]
        q = rot # rotation in quaternions
        euler_angles = euler_from_quaternion([q[0], q[1], q[2], q[3]])
        map_heading = euler_angles[2]

        # Get desired map heading
        desired_map_heading = msg.course

        # Get desired true heading
        desired_true_heading = desired_map_heading - map_heading + self.true_heading
        desired_true_heading = fmod(fmod(desired_true_heading, 2.0*pi) + 2.0*pi, 2.0*pi) # wrap to [0, 2*pi]

        # Send true heading to OBS
        mr_pb_msg = mr_ros_pb2.SpeedCourse()
        mr_pb_msg.speed = msg.speed
        mr_pb_msg.course = 2*pi - desired_true_heading # Z-axis down in OBS and up in ROS

        # Magnetic declination: Trondheim harbor
        mr_pb_msg.course = 2*pi - desired_true_heading - 0.063759321

        print('Sending SpeedCourse message: %s' % mr_pb_msg)

        string = mr_pb_msg.SerializeToString()
        self.sock.sendto(string, self.host)   

    def handleImuMsg(self, msg):
        q = msg.orientation
        euler_angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.true_heading = euler_angles[2]



if __name__ == '__main__':
    node_name = os.path.splitext(os.path.basename(__file__))[0]  # file name
    rospy.init_node(node_name)
    rospy.loginfo('[%s] running...' % node_name)
    node = mr_obs_connector()
    rospy.spin()
    rospy.loginfo('[%s] shutting down...' % node_name)
