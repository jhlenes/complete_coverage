#include <guidance/guidance.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <queue>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

namespace otter_coverage {

    Guidance::Guidance() {

        ros::NodeHandle nh;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Rate rate(10.0);
        while (nh.ok()) {

            // get the pose of the robot in the map frame
            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0), ros::Duration(0.0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("Transform from map to base_link not found.");
                continue;
            }
            double x = transformStamped.transform.translation.x;
            double y = transformStamped.transform.translation.y;
            tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                             transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            double psi = tf::getYaw(q);



            ros::spinOnce();
            rate.sleep();
        }

    }

    Guidance::~Guidance() {

    }

}
