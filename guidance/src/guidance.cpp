#include <guidance/guidance.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

namespace otter_coverage
{

Guidance::Guidance()
{
  ros::NodeHandle nh;

  //ros::Subscriber waypointSub =
  //    nh.subscribe("move_base_simple/goal", 1000, &Guidance::newWaypoint, this);

  ros::Subscriber dubinsPathSub =
      nh.subscribe("simple_dubins_path", 1000, &Guidance::newPath, this);

  m_cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    // Get the pose of the robot in the map frame
    geometry_msgs::TransformStamped tfStamped;
    try
    {
      tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                           ros::Duration(0.0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Transform from map to base_link not found: %s", ex.what());
      continue;
    }
    double x = tfStamped.transform.translation.x;
    double y = tfStamped.transform.translation.y;
    double psi = tf2::getYaw(tfStamped.transform.rotation);

    followPath(x, y, psi);

    ros::spinOnce();
    rate.sleep();
  }
}

Guidance::~Guidance() {}

void Guidance::newWaypoint(const geometry_msgs::PoseStamped& waypoint)
{
  // Add new waypoint to the list of waypoints
  m_waypoints.poses.push_back(waypoint);

  // When we get two waypoints we can start to navigate
  if (m_waypoints.poses.size() == 2)
  {
    m_currentWp = 1;
  }
}

void Guidance::newPath(const nav_msgs::Path& path) { m_path = path; }

void Guidance::followPath(double x, double y, double psi)
// TODO: cuts turns, how to fix?
{
#if 1
  // Finished?
  if (m_path.poses.size() <= 1)
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    m_cmdVelPub.publish(cmd_vel);
    return;
  }

  // Identify closest point on path
  std::vector<geometry_msgs::PoseStamped>::iterator closest;
  double minDist = std::numeric_limits<double>::max();
  for (auto it = m_path.poses.begin(); it != m_path.poses.end(); it++)
  {
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                            std::pow(y - it->pose.position.y, 2));
    if (dist < minDist)
      closest = it;
  }

  // Store closest
  geometry_msgs::PoseStamped pose_d = *closest;

  // Erase previous elements
  // m_path.poses.erase(m_path.poses.begin(), closest);

  // Path tangential angle
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);

  // Cross-track error
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) +
               (y - pose_d.pose.position.y) * std::cos(gamma_p);

  // Time-varying lookahead distance
  double delta_y_e =
      (delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) +
      delta_min;
  // if turning => small lookahead distance
  if ((closest + 1) != m_path.poses.end())
  {
    if (std::fabs(gamma_p - tf2::getYaw((*(closest + 1)).pose.orientation)) <
        std::numeric_limits<double>::epsilon())
    {
      delta_y_e = delta_min;
    }
  }

  // velocity-path relative angle
  double chi_r = std::atan(-y_e / delta_y_e);

  // desired course angle
  double chi_d = gamma_p + chi_r;

  // calculate desired yaw rate
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }
  double r = std::min(chi_err, 0.8);
  r = std::max(r, -0.8);

  // calculate desired speed
  double u = 0.4 * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, 0.1);

  // publish angle and speed
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = u;
  cmd_vel.angular.z = r;
  m_cmdVelPub.publish(cmd_vel);
#endif

#if 0
  // Not enough waypoints to navigate from
  if (m_currentWp < 1 || m_currentWp >= m_waypoints.poses.size())
  {
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
      std::pow(R, 2))
  {
    m_currentWp++;
  }

  // desired course angle
  double chi_d = alpha_k + chi_r;

  // calculate desired yaw rate
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
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
#endif
}

} // namespace otter_coverage
