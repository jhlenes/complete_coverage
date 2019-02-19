#include <guidance/guidance.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>

#include <math.h>
#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>

namespace otter_coverage {

Guidance::Guidance() {
  ros::NodeHandle nh;

  ros::Subscriber waypointSub =
      nh.subscribe("move_base_simple/goal", 1000, &Guidance::newWaypoint, this);
  ros::Subscriber pathSub =
      nh.subscribe("simple_dubins_path", 1000, &Guidance::newPath, this);
  m_cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (nh.ok()) {
    // get the pose of the robot in the map frame
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer.lookupTransform(
          "map", "base_link", ros::Time(0.0), ros::Duration(0.0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Transform from map to base_link not found.");
      continue;
    }
    double x = transformStamped.transform.translation.x;
    double y = transformStamped.transform.translation.y;
    double psi = tf::getYaw(transformStamped.transform.rotation);
    m_pose = {x, y, psi};

    followPath(x, y, psi);

    ros::spinOnce();
    rate.sleep();
  }
}

Guidance::~Guidance() {}

void Guidance::newWaypoint(const geometry_msgs::PoseStamped &waypoint) {
  // Add new waypoint to the list of waypoints
  m_waypoints.poses.push_back(waypoint);

  // When we get two waypoints we can start to navigate
  if (m_waypoints.poses.size() == 2) {
    m_currentWp = 1;
  }
}

void Guidance::newPath(const nav_msgs::Path &path) { m_path = path; }

void Guidance::followPath(double x, double y, double psi) {
#if 0
    if (m_path.poses.size() == 0) {
    return;
  }

  // Identify closest point on path
  double minDist = std::numeric_limits<double>::max();
  typedef std::vector<double>::size_type sz;
  sz minIndex = 0;
  for (sz i = 0; i < m_path.poses.size(); i++) {
    geometry_msgs::PoseStamped pose = m_path.poses[i];
    double dist = std::sqrt(std::pow(m_pose.x - pose.pose.position.x, 2) +
                            std::pow(m_pose.y - pose.pose.position.y, 2));
    if (dist < minDist) {
      minDist = dist;
      minIndex = i;
    }
  }

  // Find setpoint on path
  sz setpointIndex = minIndex + 20 * 1;
  if (setpointIndex > m_path.poses.size()) {
    setpointIndex = m_path.poses.size() - 1;
  }
#endif

  // Not enough waypoints to navigate from
  if (m_currentWp < 1 || m_currentWp >= m_waypoints.poses.size()) {
    // publish angle and speed
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    this->m_cmdVelPub.publish(cmd_vel);
    return;
  }

  // Get the relevant waypoints
  geometry_msgs::PoseStamped p1 = m_waypoints.poses.at(m_currentWp);
  geometry_msgs::PoseStamped p0 = m_waypoints.poses.at(m_currentWp - 1);

  // path tangential angle
  double alpha_k = std::atan2(p1.pose.position.y - p0.pose.position.y,
                              p1.pose.position.x - p0.pose.position.x);

  // cross track error
  double e = -(x - p0.pose.position.x) * std::sin(alpha_k) +
             (y - p0.pose.position.y) * std::cos(alpha_k);

  // velocity-path relative angle
  double chi_r = std::atan(-e / DELTA);

  // switch waypoints if close enough
  if (std::pow(p1.pose.position.x - x, 2) +
          std::pow(p1.pose.position.y - y, 2) <
      std::pow(R, 2)) {
    m_currentWp++;
  }

  // desired course angle
  double chi_d = alpha_k + chi_r;

  // calculate desired yaw rate
  double chi_err = chi_d - psi;
  while (chi_err > PI) {
    chi_err -= 2 * PI;
  }
  while (chi_err < -PI) {
    chi_err += 2 * PI;
  }
  double r = std::min(chi_err, 1.0);
  r = std::max(r, -1.0);

  ROS_INFO_STREAM("PSI: " << psi);
  ROS_INFO_STREAM("chi_d: " << chi_d);

  // calculate desired speed
  double u = 0.4 * (1 - std::abs(e) / 5 - std::abs(chi_err) / M_PI);
  u = std::max(u, 0.1);

  // publish angle and speed
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = u;
  cmd_vel.angular.z = r;
  this->m_cmdVelPub.publish(cmd_vel);
}

}  // namespace otter_coverage
