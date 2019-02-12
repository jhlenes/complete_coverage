#ifndef COVERAGE_BINN_H_
#define COVERAGE_BINN_H_

#include <coverage_binn/partition_binn.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <simple_dubins_path/simple_dubins_path.h>
#include <tf2_ros/transform_listener.h>

class CoverageBinn {
 public:
  CoverageBinn();

 private:
  void onMapReceived(const nav_msgs::OccupancyGrid& grid);
  void mainLoop(ros::NodeHandle& nh);
  bool updateRobotPose(const tf2_ros::Buffer& tfBuffer);
  void BINN();
  void evolveNeuralNetwork(double deltaTime);
  double calculateI(PartitionBinn::CellStatus status, bool covered);
  double calculateWeightSum(int l, int k);
  double calculateWeight(int l0, int k0, int l1, int k1);
  void getNeighbors(int l, int k, std::vector<PartitionBinn::Point>& neighbors);
  void findNextPos(double& xNext, double& yNext);
  double scoreFunction(double neuralActivity, double yaw, double targetYaw);

  struct Pose {
    double x;
    double y;
    double yaw;
  };

  bool m_mapInitialized;
  PartitionBinn m_partition;
  Pose m_pose;

  // Neural network params
  const double m_A = 50.0;
  const double m_B = 0.1;
  const double m_D = 0.1;
  const double m_E = 100.0;
  const double m_mu = 1.0;
  const double m_lambda = 0.1;

  SimpleDubinsPath m_dubin;
};

#endif
