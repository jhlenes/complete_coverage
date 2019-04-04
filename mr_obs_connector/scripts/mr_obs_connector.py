#!/usr/bin/env python
import os
import socket
import rospy
import mr_ros_pb2
from usv_msgs.msg import SpeedCourse
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import euler_from_quaternion


class mr_obs_connector:

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        ip = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 10010)
        self.host = (ip, port)

        self.course_diff_set = False
        self.course_diff = 0

        rospy.Subscriber("speed_course", SpeedCourse, self.handleSpeedCourseMsg)
        rospy.Subscriber("gps/heading", QuaternionStamped, self.handleQuaternion)


    def handleSpeedCourseMsg(self, msg):
        mr_pb_msg = mr_ros_pb2.SpeedCourse()
        mr_pb_msg.speed = msg.speed
        mr_pb_msg.course = msg.course + self.course_diff

        print("Sending SpeedCourse message: %s" % mr_pb_msg)

        string = mr_pb_msg.SerializeToString()
        self.sock.sendto(string, self.host)


    def handleQuaternion(self, msg):
        # ROS has its own coordinate frame which does not necessarilly have true north as 0 heading.
        # We want to output true course, thus we store the offset as the first received true heading.
        # Note: this node must be started at the same time as the SLAM node
        if not self.course_diff_set:
            self.course_diff_set = True
            
            q = msg.quaternion
            euler_angles = euler_from_quaternion([q.x, q.y, q.z, q.w])
            course = euler_angles[2]
            self.course_diff = course

            print("True starting course is: %s" % course)
            


if __name__ == '__main__':
    node_name = os.path.splitext(os.path.basename(__file__))[0]  # file name
    rospy.init_node(node_name)
    rospy.loginfo('[%s] running...' % node_name)
    node = mr_obs_connector()
    rospy.spin()
    rospy.loginfo('[%s] shutting down...' % node_name)
