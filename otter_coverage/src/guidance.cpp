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

        ros::Subscriber waypointSub = nh.subscribe("waypoint", 1000, &Guidance::newWaypoint, this);

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

            followPath(x, y, psi);

            ros::spinOnce();
            rate.sleep();
        }

    }

    Guidance::~Guidance() {

    }

    void Guidance::newWaypoint(const geometry_msgs::PoseStamped &waypoint) {

        // Add new waypoint to the list of waypoints
        waypoints.poses.push_back(waypoint);

        // When we get two waypoints we can start to navigate
        if (waypoints.poses.size() == 2) {
            currentWp = 1;
        }
    }

    void Guidance::followPath(double x, double y, double psi) {

        // Not enough waypoints to navigate from
        if (currentWp < 1) {
            return;
        }

        // Get the relevant waypoints
        geometry_msgs::PoseStamped p1 = waypoints.poses.at(currentWp);
        geometry_msgs::PoseStamped p0 = waypoints.poses.at(currentWp - 1);

        // path tangential angle
        double alpha_k = std::atan2(p1.pose.position.y - p0.pose.position.y, p1.pose.position.x - p0.pose.position.x);

        // cross track error
        double e = -(x - p0.pose.position.x) * std::sin(alpha_k) + (y - p0.pose.position.y) * std::cos(alpha_k);


    }

}
