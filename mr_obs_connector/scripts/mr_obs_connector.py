#!/usr/bin/env python
import os
import socket
import rospy
import mr_ros_pb2
from usv_msgs.msg import SpeedCourse


class mr_obs_connector:

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber("speed_course", SpeedCourse, self.handleSpeedCourseMsg)
        ip = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 10010)
        self.host = (ip, port)

    def handleSpeedCourseMsg(self, msg):
        mr_pb_msg = mr_ros_pb2.SpeedCourse()
        mr_pb_msg.speed = msg.speed
        mr_pb_msg.course = msg.course

        print("Sending SpeedCourse message: %s" % mr_pb_msg)

        string = mr_pb_msg.SerializeToString()
        self.sock.sendto(string, self.host)


if __name__ == '__main__':
    node_name = os.path.splitext(os.path.basename(__file__))[0]  # file name
    rospy.init_node(node_name)
    rospy.loginfo('[%s] running...' % node_name)
    node = mr_obs_connector()
    rospy.spin()
    rospy.loginfo('[%s] shutting down...' % node_name)
