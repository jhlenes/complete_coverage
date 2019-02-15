#ifndef OTTER_COVERAGE_H_
#define OTTER_COVERAGE_H_

#include <ros/ros.h>

#include <coverage/partition.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

namespace otter_coverage {

class Coverage {
 public:
  Coverage();

 private:
  struct Pose {
    double x;
    double y;
    double psi;
  };

  struct Goal {
    bool exists;
    bool isNew;
    int x;
    int y;
  };

  struct tile {
    int x;
    int y;
  };

  void mapCallback(const nav_msgs::OccupancyGrid &m_grid);
  void mainLoop(ros::NodeHandle nh);
  bool updatePose(const tf2_ros::Buffer &tfBuffer);
  void boustrophedonMotion();
  void checkGoal(Goal &goal);
  void checkDirection(int xOffset, int yOffset, int tileX, int tileY,
                      Goal &goal);
  bool isFree(int xTile, int yTile, bool allowUnknown);
  bool locateBestBacktrackingPoint(int &goalX, int &goalY, int tileX,
                                   int tileY);
  bool blockedOrCovered(int i, int j);
  bool isBacktrackingPoint(int i, int j);
  void publishGoal(int tileY, int tileX, Goal goal);
  double dist(double x0, double y0, double x1, double y1);

  // ROS parameters
  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;

  double m_scanRange;
  double m_tileResolution;
  double m_goalTolerance;

  Partition m_partition;

  bool m_mapInitialized;

  ros::Publisher m_goalPub;
  ros::Publisher m_pathPub;
  ros::Publisher m_dubinPub;

  nav_msgs::Path m_coveredPath;

  Pose m_pose;
};

}  // namespace otter_coverage

#endif
