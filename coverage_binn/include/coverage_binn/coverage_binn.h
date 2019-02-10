#ifndef COVERAGE_BINN_H_
#define COVERAGE_BINN_H_

#include <coverage_binn/partition_binn.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class CoverageBinn {
 public:
  CoverageBinn();

 private:
  void onMapReceived(const nav_msgs::OccupancyGrid& grid);
  void mainLoop(ros::NodeHandle& nh);
  bool updateRobotPose(const tf2_ros::Buffer& tfBuffer);
  void BINN();

  struct Pose {
    double x;
    double y;
    double yaw;
  };

  bool m_mapInitialized;
  PartitionBinn m_partition;
  Pose m_pose;
};

#endif
