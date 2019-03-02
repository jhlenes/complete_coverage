#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_filter");
  ros::NodeHandle nh;

  ros::Publisher pub =
      nh.advertise<sensor_msgs::LaserScan>("filtered_scan", 10);
  ros::Subscriber laser_sub = nh.subscribe(
      "scan", 1000,
      boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)>(
          [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
            sensor_msgs::LaserScan new_msg = *msg;
            for (auto& range : new_msg.ranges)
            {
              if (std::isinf(range) || range > new_msg.range_max)
              {
                range = new_msg.range_max - 0.01f;
              }
            }
            pub.publish(new_msg);
          }));
  ros::spin();
  return 0;
}
