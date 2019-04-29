#ifndef OTTER_COVERAGE_H_
#define OTTER_COVERAGE_H_

#include <ros/ros.h>

#include <coverage/a_star.h>
#include <coverage/partition.h>

#include <deque>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

#include <simple_dubins_path/simple_dubins_path.h>

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
  bool updatePose();
  void boustrophedonCoverage(int gx, int gy, Goal goal);
  Goal updateWPs(int gx, int gy);
  bool checkDirection(Direction dir, int gx, int gy);
  bool isFree(int gx, int gy, bool allowUnknown);
  bool locateBestBacktrackingPoint(int& goalX, int& goalY, int gx, int gy, std::vector<Tile> &bestPath);
  bool blockedOrCovered(int gx, int gy);
  bool freeAndNotCovered(int gx, int gy);
  bool isBacktrackingPoint(int gx, int gy, Tile &bp);
  void publishGoal(int gx, int gy, Goal goal);
  void newTrack(int gx, int gy);

  // ROS parameters
  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;
  double m_scanRange;
  double m_tileResolution;
  double m_goalTolerance;

  bool m_mapInitialized;
  bool m_dirInitialized = false;

  Direction m_dir = North;
  Direction m_sweepDir = East;

  int m_trackX = 0;
  int m_trackY = 0;

  bool m_wallFollowing = false;
  bool m_backtracking = false;
  bool m_finished = false;

  ros::Publisher m_goalPub;
  ros::Publisher m_pathPub;
  ros::Publisher m_dubinPub;

  nav_msgs::Path m_coveredPath;

  std::deque<Tile> m_waypoints;

  Partition m_partition;

  Pose m_pose;

  int m_coverageSize;
  int m_minCoverageSize = -1;
};

} // namespace otter_coverage

#endif
