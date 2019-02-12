#include <coverage_binn/coverage_binn.h>
#include <tf2/utils.h>
#include <cmath>

CoverageBinn::CoverageBinn() : m_mapInitialized(false) {
  ROS_INFO("coverage_binn_node started.");

  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");

  // Get parameters
  ROS_INFO("Starting main loop.");

  // Set up partition. TODO: set up with parameters
  m_partition = PartitionBinn(nh);
  m_partition.initialize(-15, -10, 15, 10, 1.5);
  ROS_INFO("Starting main loop.");

  // Set up subscribers
  ros::Subscriber mapSub =
      nh.subscribe("inflated_map", 1000, &CoverageBinn::onMapReceived, this);

  // Set up publishers

  // Start main loop
  ROS_INFO("Starting main loop.");
  mainLoop(nh);
}

void CoverageBinn::onMapReceived(const nav_msgs::OccupancyGrid& grid) {
  if (!m_mapInitialized) m_mapInitialized = true;
  m_partition.update(grid, m_pose.x, m_pose.y);
}

void CoverageBinn::mainLoop(ros::NodeHandle& nh) {
  // Set up tf2 transform listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Loop
  ros::Rate rate(10.0);
  while (nh.ok()) {
    if (!updateRobotPose(tfBuffer)) {
      ros::Duration(1.0).sleep();
      continue;
    }

    // Do BINN
    BINN();

    ros::spinOnce();
    rate.sleep();
  }
}

bool CoverageBinn::updateRobotPose(const tf2_ros::Buffer& tfBuffer) {
  geometry_msgs::TransformStamped tfStamped;
  try {
    // TODO: use params for frames
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                         ros::Duration(0.1));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // Get pose of robot in map frame
  m_pose.x = tfStamped.transform.translation.x;
  m_pose.y = tfStamped.transform.translation.y;
  m_pose.yaw = tf2::getYaw(tfStamped.transform.rotation);

  return true;
}

void CoverageBinn::BINN() {
  if (!m_mapInitialized) return;

  static ros::Time lastTime = ros::Time::now();
  double deltaTime = (ros::Time::now() - lastTime).toSec();
  lastTime = ros::Time::now();

  // Are we finished?
  // TODO: check for free, uncovered cells

  // Update neural activity of each cell
  // TODO: figure out why deltaTime by itself is too large
  // TODO: doesn't work with higher deltaTime
  evolveNeuralNetwork(deltaTime / 10.0);

  // Find next position
  double xNext, yNext;
  findNextPos(xNext, yNext);

  // Set current cell as covered
  int l, k;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l, k);
  m_partition.setCellCovered(l, k, true);
}

void CoverageBinn::evolveNeuralNetwork(double deltaTime) {
  auto cells = m_partition.getCells();

  for (int l = 1; l <= cells.size(); l++) {
    auto column = cells[l - 1];
    for (int k = 1; k <= column.size(); k++) {
      double I = calculateI(m_partition.getCellStatus(l, k),
                            m_partition.isCellCovered(l, k));

      double weightSum =
          calculateWeightSum(l, k, m_partition.getCellValue(l, k));

      double x = m_partition.getCellValue(l, k);
      double xDot = -m_A * x + (m_B - x) * (std::max(I, 0.0) + weightSum) -
                    (m_D + x) * std::max(-I, 0.0);

      x += xDot * deltaTime;
      m_partition.setCellValue(l, k, x);  // std::max(std::min(x, m_B), m_D)
    }
  }
}

double CoverageBinn::calculateI(PartitionBinn::CellStatus status,
                                bool covered) {
  // Scaling factor for target priorities: 0 < lambda <= 1
  double lambda = 1;  // TODO: Prioritize targets smarter

  if (status == PartitionBinn::Free && !covered) {  // target
    return lambda * m_E;
  } else if (status == PartitionBinn::Blocked) {  // obstacle
    return -m_E;
  } else {
    return 0.0;
  }
}

double CoverageBinn::calculateWeightSum(int l, int k, double x) {
  std::vector<PartitionBinn::Point> neighbors;
  getNeighbors(l, k, neighbors);

  double weightSum = 0.0;
  for (auto neighbor : neighbors) {
    if (neighbor.l < 1 || neighbor.l > m_partition.getCells().size() ||
        neighbor.k < 1 ||
        neighbor.k > m_partition.getCells()[neighbor.l - 1].size()) {
      continue;
    }
    weightSum +=
        calculateWeight(l, k, neighbor.l, neighbor.k) * std::max(x, 0.0);
  }

  return weightSum;
}

// TODO: can be calculated offline and stored
double CoverageBinn::calculateWeight(int l0, int k0, int l1, int k1) {
  double x0, y0;
  m_partition.gridToWorld(l0, k0, x0, y0);
  double x1, y1;
  m_partition.gridToWorld(l1, k1, x1, y1);
  return m_mu / std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

void CoverageBinn::getNeighbors(int l, int k,
                                std::vector<PartitionBinn::Point>& neighbors) {
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
  if (l % 2 == 1) {
    neighbors.push_back({l + 1, k - 2});
    neighbors.push_back({l - 1, k - 2});
  } else {  // l % 2 == 0
    neighbors.push_back({l + 1, k + 2});
    neighbors.push_back({l - 1, k + 2});
  }
}

void CoverageBinn::findNextPos(double& xNext, double& yNext) {
  // Get cell position
  int l, k;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l, k);

  // Next pos has to be among the neighbors
  std::vector<PartitionBinn::Point> neighbors;
  getNeighbors(l, k, neighbors);

  // Consider current cell first
  PartitionBinn::Point best = {l, k};
  double maxValue =
      valueFunction(m_partition.getCellValue(l, k), m_pose.yaw, m_pose.yaw);

  // Find neighbor with highest score
  for (auto nb : neighbors) {
    // Curremt cell position
    double xCurrent, yCurrent;
    m_partition.gridToWorld(l, k, xCurrent, yCurrent);

    // Get target heading
    double xTarget, yTarget;
    m_partition.gridToWorld(nb.l, nb.k, xTarget, yTarget);
    double targetHeading = m_dubin.getTargetHeading(
        m_pose.x, m_pose.y, m_pose.yaw, xTarget, yTarget);

    double value = valueFunction(m_partition.getCellValue(nb.l, nb.k),
                                 m_pose.yaw, targetHeading);
    if (value > maxValue) {
      maxValue = value;
      best = nb;
    }
  }
  ROS_INFO_STREAM("Current pos: " << l << ", " << k);
  ROS_INFO_STREAM("Next pos:    " << best.l << ", " << best.k);
  xNext = best.l;
  yNext = best.k;
}

double CoverageBinn::valueFunction(double neuralActivity, double yaw,
                                   double targetYaw) {
  double diff = std::fabs(targetYaw - yaw);
  diff = std::fmod(diff, M_PI);

  return (1 - diff / M_PI) * m_lambda * neuralActivity +
         (1 - m_lambda) * neuralActivity;
}
