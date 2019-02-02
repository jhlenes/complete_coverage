#ifndef SIMPLE_DUBINS_PATH_H_
#define SIMPLE_DUBINS_PATH_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace otter_coverage {

class SimpleDubinsPath {
 public:
  SimpleDubinsPath();
  ~SimpleDubinsPath();

  bool makePath(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path);

 private:
  void onGoal(const geometry_msgs::PoseStamped& goal);

  enum TurningDirection { Left = 1, Right = -1 };

  double m_turningRadius;
  double m_pathResolution;

  ros::Publisher m_pathPub;
};

}  // namespace otter_coverage

#endif
