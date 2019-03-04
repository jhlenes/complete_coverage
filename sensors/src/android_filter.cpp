#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_filter");
  ros::NodeHandle nh;

  // IMU
  ros::Publisher pubImu = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
  ros::Subscriber laser_sub = nh.subscribe(
      "android/imu", 1000,
      boost::function<void(const sensor_msgs::Imu::ConstPtr&)>(
          [&](const sensor_msgs::Imu::ConstPtr& msg) {

            // Change frame id
            sensor_msgs::Imu new_msg = *msg;
            new_msg.header.frame_id = "imu_link";
            new_msg.header.stamp = ros::Time::now();
            pubImu.publish(new_msg);
          }));

  // GPS
  ros::Publisher pubGps = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 10);
  ros::Subscriber gps_sub = nh.subscribe(
      "android/fix", 1000,
      boost::function<void(const sensor_msgs::NavSatFix::ConstPtr&)>(
          [&](const sensor_msgs::NavSatFix::ConstPtr& msg) {

            // Change frame id
            sensor_msgs::NavSatFix new_msg = *msg;
            new_msg.header.frame_id = "gps_link";
            new_msg.header.stamp = ros::Time::now();
            pubGps.publish(new_msg);
          }));

  ros::spin();
  return 0;
}
