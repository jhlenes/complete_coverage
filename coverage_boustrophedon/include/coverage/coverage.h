#ifndef OTTER_COVERAGE_H_
#define OTTER_COVERAGE_H_

#include <ros/ros.h>

#include <coverage/a_star.h>
#include <coverage/partition.h>

#include <deque>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

namespace otter_coverage
{

class Coverage
{
public:
  Coverage();

private:
  struct Pose
  {
    double x;
    double y;
    double psi;
  };

  struct Goal
  {
    Goal() : reached(true) {}
    Goal(Tile tile) : reached(false), gx(tile.gx), gy(tile.gy) {}
    bool reached;
    int gx;
    int gy;
  };

  enum Direction
  {
    North,
    South,
    East,
    West
  };

  void mapCallback(const nav_msgs::OccupancyGrid& m_grid);
  void mainLoop(ros::NodeHandle nh);
  bool updatePose(const tf2_ros::Buffer& tfBuffer);
  void boustrophedonCoverage(int gx, int gy, Goal goal);
  Goal updateWPs(int gx, int gy);
  bool checkDirection(Direction dir, int gx, int gy);
  bool isFree(int gx, int gy, bool allowUnknown);
  bool locateBestBacktrackingPoint(int& goalX, int& goalY, int gx, int gy);
  bool blockedOrCovered(int gx, int gy);
  bool freeAndNotCovered(int gx, int gy);
  bool isBacktrackingPoint(int gx, int gy);
  void publishGoal(int gx, int gy, Goal goal);

  bool m_mapInitialized;

  // ROS parameters
  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;
  double m_scanRange;
  double m_tileResolution;
  double m_goalTolerance;

  ros::Publisher m_goalPub;
  ros::Publisher m_pathPub;
  ros::Publisher m_dubinPub;

  nav_msgs::Path m_coveredPath;

  std::deque<Tile> m_waypoints;

  Partition m_partition;
  Pose m_pose;
};

} // namespace otter_coverage

#endif
