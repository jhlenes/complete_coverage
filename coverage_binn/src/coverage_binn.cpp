#include <coverage_binn/coverage_binn.h>
#include <tf2/utils.h>

CoverageBinn::CoverageBinn() : m_mapInitialized(false) {
  ROS_INFO("coverage_binn_node started.");

  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");

  // Get parameters
  ROS_INFO("Starting main loop.");

  // Set up partition. TODO: set up with parameters
  m_partition = PartitionBinn();
  m_partition.initialize(-20, -20, 20, 20, 1.5);
  ROS_INFO("Starting main loop.");

  // Set up subscribers
  ros::Subscriber mapSub =
      nh.subscribe("coverage_map", 1000, &CoverageBinn::onMapReceived, this);

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
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0));
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
  // if (!m_mapInitialized) return;
  int l;
  int k;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l, k);
  ROS_INFO_STREAM("Row: " << l << " Col: " << k);

  // Are we finished?
  // TODO: check for free, uncovered cells

  // Evolve neural network

  // Find next position

  // Set current cell as covered
}
