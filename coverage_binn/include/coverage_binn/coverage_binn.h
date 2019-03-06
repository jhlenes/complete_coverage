#ifndef COVERAGE_BINN_H_
#define COVERAGE_BINN_H_

#include <coverage_binn/partition_binn.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <coverage_binn/simple_dubins_path.h>
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
  double calculateI(PartitionBinn::CellStatus status, bool covered, double x,
					double y);
  double calculateWeightSum(int l, int k);
  double calculateWeight(int l0, int k0, int l1, int k1);
  void getNeighbors(int l, int k, std::vector<PartitionBinn::Point>& neighbors);
  void getNeighbors2(int l, int k,
					 std::vector<PartitionBinn::Point>& neighbors);
  void findNextCell(int& lNext, int& kNext, double& yawNext);
  double scoreFunction(double neuralActivity, double yaw, double targetYaw);
  void publishGoal(double x, double y, double yaw);

  struct Pose {
    double x;
    double y;
    double yaw;
  };

  bool m_mapInitialized;
  PartitionBinn m_partition;
  Pose m_pose;

  // Partition
  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;

  // Neural network params
  const double m_A = 50.0;
  const double m_B = 0.1;
  const double m_D = 0.1;
  const double m_E = 100.0;
  const double m_mu = 1.0;
  const double m_lambda = 0.1;

  // Params
  double m_circleAcceptance = 1.0;

  coverage_binn::SimpleDubinsPath m_dubin;

  ros::Publisher m_goalPub;
  ros::Publisher m_dubinPub;

};

#endif
