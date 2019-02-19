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

  struct Pose
  {
    double x;
    double y;
    double psi;
  };

  Pose m_pose;
  nav_msgs::Path m_path;

  int m_currentWp;
  nav_msgs::Path m_waypoints;

  ros::Publisher m_cmdVelPub;

  const double PI = std::atan(1.0) * 4;

  // lookahead distance
  double DELTA = 2.0;

  // circle of acceptance
  double R = 1.0;
};

} // namespace otter_coverage

#endif
