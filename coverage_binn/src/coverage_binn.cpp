#include <cmath>
#include <coverage_binn/coverage_binn.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

#include <coverage_boustrophedon/DubinInput.h>

double pointDistance(double x0, double y0, double x1, double y1)
{
  return std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

CoverageBinn::CoverageBinn() : m_mapInitialized(false)
{
  ROS_INFO("coverage_binn_node started.");

  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");

  // Get parameters
  m_x0 = nhP.param("x0", -51.0);
  m_y0 = nhP.param("y0", -51.0);
  m_x1 = nhP.param("x1", 100.0);
  m_y1 = nhP.param("y1", 50.0);
  double cellRadius = nhP.param("cell_radius", 2.5);
  double scanRange = nhP.param("scan_range", 12);
  m_circleAcceptance = nhP.param("goal_tolerance", 3.0);

  // Set up partition. TODO: set up with parameters
  m_partition = PartitionBinn(nh);
  m_partition.initialize(m_x0, m_y0, m_x1, m_y1, cellRadius, scanRange);

  // Set up subscribers
  ros::Subscriber mapSub =
      nh.subscribe("inflated_map", 1000, &CoverageBinn::onMapReceived, this);

  // Set up publishers
  m_goalPub =
      nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  m_dubinPub = nh.advertise<coverage_boustrophedon::DubinInput>(
      "simple_dubins_path/input", 1000);

  // Start main loop
  ROS_INFO("Starting main loop.");
  mainLoop(nh);
}

void CoverageBinn::onMapReceived(const nav_msgs::OccupancyGrid& grid)
{
  if (!m_mapInitialized)
    m_mapInitialized = true;
  m_partition.update(grid, m_pose.x, m_pose.y);
}

void CoverageBinn::mainLoop(ros::NodeHandle& nh)
{
  // Set up tf2 transform listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Loop
  ros::Rate rate(5.0);
  while (nh.ok())
  {
    if (!updateRobotPose(tfBuffer))
    {
      ros::Duration(1.0).sleep();
      continue;
    }

    // Do BINN
    BINN();

    ros::spinOnce();
    rate.sleep();
  }
}

bool CoverageBinn::updateRobotPose(const tf2_ros::Buffer& tfBuffer)
{
  geometry_msgs::TransformStamped tfStamped;
  try
  {
    // TODO: use params for frames
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                         ros::Duration(0.1));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // Get pose of robot in map frame
  m_pose.x = tfStamped.transform.translation.x;
  m_pose.y = tfStamped.transform.translation.y;
  m_pose.yaw = tf2::getYaw(tfStamped.transform.rotation);

  return true;
}

void CoverageBinn::BINN()
{
  if (!m_mapInitialized)
    return;

  // Are we finished?
  // TODO: check for free, uncovered cells
  if (m_partition.hasCompleteCoverage())
  {
    ROS_INFO("Finished!");
    return;
  }

  static ros::Time lastTime = ros::Time::now();
  double deltaTime = (ros::Time::now() - lastTime).toSec();
  lastTime = ros::Time::now();

  // Update neural activity of each cell
  // TODO: figure out why deltaTime by itself is too large
  // TODO: doesn't work with higher deltaTime
  evolveNeuralNetwork(0.01);

  // Find next position
  int lNext, kNext;
  double yawNext;
  findNextCell(lNext, kNext, yawNext);
  double xNext, yNext;
  m_partition.gridToWorld(lNext, kNext, xNext, yNext);

  // Find current position
  int l, k;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l, k);
  double xCurrent, yCurrent;
  m_partition.gridToWorld(l, k, xCurrent, yCurrent);

  // Set current cell as covered, if we're within a circle of acceptance
  if (pointDistance(m_pose.x, m_pose.y, xCurrent, yCurrent) <
      m_circleAcceptance)
  {
    m_partition.setCellCovered(l, k, true);
  }

  static int lGoal = lNext;
  static int kGoal = kNext;
  static bool first = true;
  if (lGoal != lNext || kGoal != kNext || first)
  {
    first = false;
    // Not covered, free and already in goal cell => don't switch target
    if (m_partition.getCellStatus(l, k) == PartitionBinn::Free &&
        !m_partition.isCellCovered(l, k) && lGoal == l && kGoal == k)
    {
    }
    else
    {
      lGoal = lNext;
      kGoal = kNext;
      publishGoal(xNext, yNext, yawNext);
    }
  }

  ROS_INFO_STREAM("Current pos: " << l << ", " << k);
  ROS_INFO_STREAM("Next pos:    " << lNext << ", " << kNext << ", " << yawNext);
}

void CoverageBinn::evolveNeuralNetwork(double deltaTime)
{
  auto cells = m_partition.getCells();

  for (int l = 1; l <= cells.size(); l++)
  {
    auto column = cells[l - 1];
    for (int k = 1; k <= column.size(); k++)
    {
      double xCell, yCell;
      m_partition.gridToWorld(l, k, xCell, yCell);
      double I = calculateI(m_partition.getCellStatus(l, k),
                            m_partition.isCellCovered(l, k), xCell, yCell);

      double weightSum = calculateWeightSum(l, k);

      double x = m_partition.getCellValue(l, k);

      double xDot = -m_A * x + (m_B - x) * (std::max(I, 0.0) + weightSum) -
                    (m_D + x) * std::max(-I, 0.0);

      x += xDot * deltaTime;
      m_partition.setCellValue(l, k, x);
    }
  }
}

double CoverageBinn::calculateI(PartitionBinn::CellStatus status, bool covered,
                                double x, double y)
{
  // Scaling factor for target priorities: 0 < lambda <= 1
  // TODO: Prioritize targets smarter, maybe do this with score function
  // instead.
  double lambda = 1;
  // lambda -= 0.9 * (1 - (x + m_x0) / (m_x1 - m_x0));

  if (status == PartitionBinn::Free && !covered)
  { // target
    return lambda * m_E;
  }
  else if (status == PartitionBinn::Blocked)
  { // obstacle
    return -m_E;
  }
  else
  {
    return 0.0;
  }
}

double CoverageBinn::calculateWeightSum(int l, int k)
{
  std::vector<PartitionBinn::Point> neighbors;
  getNeighbors2(l, k, neighbors);

  double weightSum = 0.0;
  for (auto nb : neighbors)
  {
    if (nb.l < 1 || nb.l > m_partition.getCells().size() || nb.k < 1 ||
        nb.k > m_partition.getCells()[nb.l - 1].size())
    {
      continue;
    }

    weightSum += calculateWeight(l, k, nb.l, nb.k) *
                 std::max(m_partition.getCellValue(nb.l, nb.k), 0.0);
  }

  return weightSum;
}

// TODO: can be calculated offline and stored
double CoverageBinn::calculateWeight(int l0, int k0, int l1, int k1)
{
  double x0, y0;
  m_partition.gridToWorld(l0, k0, x0, y0);
  double x1, y1;
  m_partition.gridToWorld(l1, k1, x1, y1);

  return m_mu / std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

void CoverageBinn::getNeighbors(int l, int k,
                                std::vector<PartitionBinn::Point>& neighbors)
{
  neighbors.push_back({l + 1, k});
  neighbors.push_back({l + 1, k + 1});
  neighbors.push_back({l + 1, k - 1});
  neighbors.push_back({l - 1, k});
  neighbors.push_back({l - 1, k + 1});
  neighbors.push_back({l - 1, k - 1});
  neighbors.push_back({l + 2, k});
  neighbors.push_back({l - 2, k});
  neighbors.push_back({l, k + 1});
  neighbors.push_back({l, k - 1});
  if (l % 2 == 1)
  {
    neighbors.push_back({l + 1, k - 2});
    neighbors.push_back({l - 1, k - 2});
  }
  else
  { // l % 2 == 0
    neighbors.push_back({l + 1, k + 2});
    neighbors.push_back({l - 1, k + 2});
  }
}

void CoverageBinn::getNeighbors2(int l, int k,
                                 std::vector<PartitionBinn::Point>& neighbors)
{
  neighbors.push_back({l + 1, k});
  neighbors.push_back({l - 1, k});
  neighbors.push_back({l, k + 1});
  neighbors.push_back({l, k - 1});
  if (l % 2 == 1)
  {
    neighbors.push_back({l + 1, k - 1});
    neighbors.push_back({l - 1, k - 1});
  }
  else
  { // l % 2 == 0
    neighbors.push_back({l + 1, k + 1});
    neighbors.push_back({l - 1, k + 1});
  }
}

void CoverageBinn::findNextCell(int& lNext, int& kNext, double& yawNext)
{
  // Get cell position
  int l, k;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l, k);

  // Next pos has to be among the neighbors
  std::vector<PartitionBinn::Point> neighbors;
  getNeighbors2(l, k, neighbors);

  // Consider current cell as well
  neighbors.push_back({l, k});

  // Find neighbor with highest score
  PartitionBinn::Point best = {l, k};
  double maxScore = std::numeric_limits<double>::lowest();
  for (auto nb : neighbors)
  {
    if (nb.l < 1 || nb.l > m_partition.getCells().size() || nb.k < 1 ||
        nb.k > m_partition.getCells()[nb.l - 1].size())
    {
      continue;
    }

    // Current cell position
    double xCurrent, yCurrent;
    m_partition.gridToWorld(l, k, xCurrent, yCurrent);

    // Target position
    double xTarget, yTarget;
    m_partition.gridToWorld(nb.l, nb.k, xTarget, yTarget);

    // Target reachable?
    double yawTarget;
    if (m_dubin.getTargetHeading(m_pose.x, m_pose.y, m_pose.yaw, xTarget,
                                 yTarget, yawTarget))
    {
      // Get the score
      double score = scoreFunction(m_partition.getCellValue(nb.l, nb.k),
                                   m_pose.yaw, yawTarget);
      if (score > maxScore)
      {
        maxScore = score;
        best = nb;
        yawNext = yawTarget;
      }
    }
  }

  lNext = best.l;
  kNext = best.k;
}

double CoverageBinn::scoreFunction(double neuralActivity, double yaw,
                                   double targetYaw)
{
  if (yaw < 0)
    yaw += 2 * M_PI;
  if (targetYaw < 0)
    targetYaw += 2 * M_PI;
  double diff = std::fabs(targetYaw - yaw);
  if (diff > M_PI)
    diff = 2 * M_PI - diff;

  return (1 - diff / M_PI) * m_lambda * neuralActivity +
         (1 - m_lambda) * neuralActivity;
}

void CoverageBinn::publishGoal(double x, double y, double yaw)
{
  // Publish goal
  geometry_msgs::PoseStamped goalPose;
  goalPose.header.stamp = ros::Time::now();
  goalPose.header.frame_id = "map";
  goalPose.pose.position.x = x;
  goalPose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goalPose.pose.orientation.x = q.x();
  goalPose.pose.orientation.y = q.y();
  goalPose.pose.orientation.z = q.z();
  goalPose.pose.orientation.w = q.w();
  m_goalPub.publish(goalPose);

  // Current pose
  geometry_msgs::PoseStamped startPose;
  startPose.header.stamp = ros::Time::now();
  startPose.header.frame_id = "map";
  startPose.pose.position.x = m_pose.x;
  startPose.pose.position.y = m_pose.y;
  startPose.pose.position.z = 0.0;
  q.setRPY(0, 0, m_pose.yaw);
  startPose.pose.orientation.x = q.x();
  startPose.pose.orientation.y = q.y();
  startPose.pose.orientation.z = q.z();
  startPose.pose.orientation.w = q.w();

  // Publish DubinInput
  coverage_boustrophedon::DubinInput di;
  di.header.stamp = ros::Time::now();
  di.header.frame_id = "map";
  di.start = startPose;
  di.end = goalPose;
  m_dubinPub.publish(di);
}
