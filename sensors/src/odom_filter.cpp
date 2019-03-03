#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;

void callback(const nav_msgs::OdometryConstPtr& msg)
{
  static int last_sec = -1;
  static int last_nsec = -1;

  int sec = msg->header.stamp.sec;
  int nsec = msg->header.stamp.nsec;

  if (last_sec > 0) {
    if (sec > last_sec || (sec == last_sec && nsec > last_nsec)) {
      odom_pub.publish(msg);

      last_sec = msg->header.stamp.sec;
      last_nsec = msg->header.stamp.nsec;
    }
  } else {
    last_sec = msg->header.stamp.sec;
    last_nsec = msg->header.stamp.nsec;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

  message_filters::Subscriber<nav_msgs::Odometry> sub(nh, "odometry/filtered", 1);
  message_filters::TimeSequencer<nav_msgs::Odometry> seq(sub, ros::Duration(0.1), ros::Duration(0.01), 10);
  seq.registerCallback(callback);

  ros::spin();

  return 0;
}
