#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace otter_coverage
{

class Guidance
{
public:
  Guidance();
  ~Guidance();

private:
  void newWaypoint(const geometry_msgs::PoseStamped& waypoint);
  void newPath(const nav_msgs::Path& path);
  void followPath(double x, double y, double psi);
  double dist(double x0, double y0, double x1, double y1) const;

  int m_currentWp;

  nav_msgs::Path m_path;

  nav_msgs::Path m_waypoints;

  ros::Publisher m_cmdVelPub;

  // lookahead distance
  double DELTA = 0.5;

  // time-varying lookahead distance
  double delta_max = 5.0;
  double delta_min = 2.0;
  double delta_k = 1.0;

  // circle of acceptance
  double R = 1.0;

  const double m_maxSpeed = 0.6;
  const double m_maxTurningRate = 0.6;
};

} // namespace otter_coverage

#endif
