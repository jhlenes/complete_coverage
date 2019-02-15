#include <coverage/coverage.h>

#include <geometry_msgs/PoseStamped.h>
#include <otter_coverage/DubinInput.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <math.h>
#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>

namespace otter_coverage {

double Coverage::dist(double x0, double y0, double x1, double y1) {
  return std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

Coverage::Coverage() {
  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");

  // Get parameters
  m_x0 = nhP.param("x0", -15);
  m_y0 = nhP.param("y0", -10);
  m_x1 = nhP.param("x1", 50);
  m_y1 = nhP.param("y1", 20);
  m_tileResolution = nhP.param("tile_resolution", 5.0);
  m_scanRange = nhP.param("scan_range", 10);
  m_goalTolerance = nhP.param("goal_tolerance", 1.0);

  // Set up partition
  m_partition = Partition(nh);
  m_partition.initialize(m_x0, m_y0, m_x1, m_y1, m_tileResolution, m_scanRange);

  // Set up subscribers
  ros::Subscriber sub =
      nh.subscribe("inflated_map", 1000, &Coverage::mapCallback, this);

  // Set up publishers
  m_goalPub =
      nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  m_pathPub = nh.advertise<nav_msgs::Path>("covered_path", 1000);
  m_dubinPub = nh.advertise<otter_coverage::DubinInput>(
      "simple_dubins_path/input", 1000);

  mainLoop(nh);
}

void Coverage::mapCallback(const nav_msgs::OccupancyGrid &grid) {
  m_partition.update(grid, m_pose.x, m_pose.y);
  if (!m_mapInitialized) m_mapInitialized = true;
}

void Coverage::mainLoop(ros::NodeHandle nh) {
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(5.0);
  while (nh.ok()) {
    if (!updatePose(tfBuffer)) {
      ROS_WARN("Transform from map to base_link not found.");
      ros::Duration(1.0).sleep();
      continue;
    }

    boustrophedonMotion();

    ros::spinOnce();
    rate.sleep();
  }
}

/**
 * @brief Coverage::updatePose Updates the pose of the robot in the map frame.
 * @param tfBuffer
 * @return true - if a transform from map to base_link was found.
 */
bool Coverage::updatePose(const tf2_ros::Buffer &tfBuffer) {
  geometry_msgs::TransformStamped tfStamped;
  try {
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                         ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  m_pose.x = tfStamped.transform.translation.x;
  m_pose.y = tfStamped.transform.translation.y;
  m_pose.psi = tf2::getYaw(tfStamped.transform.rotation);

  return true;
}

void Coverage::boustrophedonMotion() {
  // We need to wait until we have a map
  if (!m_mapInitialized) return;

  // Find tile where robot is located
  int tileX, tileY;
  m_partition.worldToGrid(m_pose.x, m_pose.y, tileX, tileY);

  static Goal goal = {true, true, tileX, tileY};

  // When initializing, move to center of start tile
  static bool initialized = false;
  if (!initialized) {
    publishGoal(tileY, tileX, goal);
    initialized = true;
  }

  // We want to go back to the start once we are finished
  static const int startX = tileX;
  static const int startY = tileY;
  static bool finished = false;

  checkGoal(goal);

  // Check to find the next tile in north-south-east-west priority.
  checkDirection(1, 0, tileX, tileY, goal);
  checkDirection(-1, 0, tileX, tileY, goal);
  checkDirection(0, -1, tileX, tileY, goal);
  checkDirection(0, 1, tileX, tileY, goal);

  // A new goal has been given manually
  if (finished && goal.isNew) {
    finished = false;
  }

  // If all directions are blocked, then the critical point has been reached
  if (!goal.exists) {
    if (locateBestBacktrackingPoint(goal.x, goal.y, tileX, tileY)) {
      ROS_INFO("Critical point! Backtracking...");
      goal.exists = true;
      goal.isNew = true;
    } else {
      if (!finished) {
        ROS_INFO("Finished! Going back to start...");
        goal.x = startX;
        goal.y = startY;
        goal.exists = true;
        goal.isNew = true;
        finished = true;
      }
    }
  }

  if (goal.isNew) {
    goal.isNew = false;
    publishGoal(tileY, tileX, goal);
  }
}

void Coverage::checkGoal(Goal &goal) {
  if (goal.exists) {
    // Check if the robot is within a circle of acceptance
    geometry_msgs::PoseStamped goalPose = m_coveredPath.poses.back();
    if (std::sqrt(std::pow(goalPose.pose.position.x - m_pose.x, 2) +
                  std::pow(goalPose.pose.position.y - m_pose.y, 2)) <
        m_goalTolerance) {
      goal.exists = false;
      goal.isNew = false;
      m_partition.setCovered(goal.x, goal.y, true);

      // Check if goal is blocked
    } else if (m_partition.getStatus(goal.x, goal.y) == Partition::Blocked) {
      ROS_INFO("Current waypoint is blocked. Searching for new... ");
      m_coveredPath.poses.erase(m_coveredPath.poses.end() - 1);
      goal.exists = false;
      goal.isNew = false;
    }
  }
}

bool Coverage::blockedOrCovered(int i, int j) {
  if (i < 0 || i >= m_partition.getWidth() || j < 0 ||
      j >= m_partition.getHeight()) {
    return false;
  }
  return m_partition.getStatus(i, j) == Partition::Blocked ||
         m_partition.isCovered(i, j);
}

bool Coverage::isBacktrackingPoint(int gx, int gy) {
  // b(s1,s8) or b(s1,s2)
  bool eastBP = false;
  if (gy - 1 >= 0) {
    eastBP =
        m_partition.getStatus(gx, gy - 1) == Partition::Free &&
        (blockedOrCovered(gx + 1, gy - 1) || blockedOrCovered(gx - 1, gy - 1));
  }

  // b(s5,s6) or b(s5,s4)
  bool westBP = false;
  if (gy + 1 < m_partition.getHeight()) {
    m_partition.getStatus(gx, gy + 1) == Partition::Free &&
        (blockedOrCovered(gx + 1, gy + 1) || blockedOrCovered(gx - 1, gy + 1));
  }

  // b(s7,s6) or b(s7,s8)
  bool southBP = false;
  if (gx - 1 >= 0) {
    southBP =
        m_partition.getStatus(gx - 1, gy) == Partition::Free &&
        (blockedOrCovered(gx - 1, gy + 1) || blockedOrCovered(gx - 1, gy - 1));
  }
  // Note: north can not be a BP because of the north-south-east-west check
  // priority.

  return eastBP || westBP || southBP;
}

bool Coverage::locateBestBacktrackingPoint(int &goalX, int &goalY, int tileX,
                                           int tileY) {
  double minDistance = -1.0;
  for (int gx = 0; gx < m_partition.getHeight(); gx++) {
    for (int gy = 0; gy < m_partition.getWidth(); gy++) {
      if (isBacktrackingPoint(gx, gy)) {
        // TODO: change to A*SPT distance
        double distance = dist(gx, gy, tileX, tileY);
        if (distance < minDistance) {
          minDistance = distance;
          goalX = gx;
          goalY = gy;
        }
      }
    }
  }

  if (minDistance < 0)
    return false;
  else
    return true;
}

void Coverage::checkDirection(int xOffset, int yOffset, int tileX, int tileY,
                              Goal &goal) {
  if (goal.exists) return;
  if (tileX + xOffset >= m_partition.getWidth() || tileX + xOffset < 0 ||
      tileY + yOffset >= m_partition.getHeight() || tileY + yOffset < 0) {
    return;
  }

  if (!m_partition.isCovered(tileX + xOffset, tileY + yOffset) &&
      m_partition.getStatus(tileX + xOffset, tileY + yOffset) ==
          Partition::Free) {
    goal.x = tileX + xOffset;
    goal.y = tileY + yOffset;
    goal.exists = true;
    goal.isNew = true;
  }
}

void Coverage::publishGoal(int tileY, int tileX, Goal goal) {
  geometry_msgs::PoseStamped goalPose;

  goalPose.header.stamp = ros::Time::now();
  goalPose.header.frame_id = "map";

  // set goal to middle of tile
  double x, y;
  m_partition.gridToWorld(goal.x, goal.y, x, y);
  goalPose.pose.position.x = x;
  goalPose.pose.position.y = y;
  goalPose.pose.position.z = 0.0;

  double yaw = std::atan2(goal.y - tileY, goal.x - tileX);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  goalPose.pose.orientation.x = q.x();
  goalPose.pose.orientation.y = q.y();
  goalPose.pose.orientation.z = q.z();
  goalPose.pose.orientation.w = q.w();

  m_goalPub.publish(goalPose);

  m_coveredPath.header.stamp = ros::Time::now();
  m_coveredPath.header.frame_id = "map";
  m_coveredPath.poses.push_back(goalPose);

  m_pathPub.publish(m_coveredPath);

  /*
  geometry_msgs::PoseStamped startPose;
  startPose.header.stamp = ros::Time::now();
  startPose.header.frame_id = "map";
  // set pose to middle of tile
  startPose.pose.position.x =
      m_pose.x;  //(tileX + 0.5 - ORIGIN_X) * m_tile_resolution;
  startPose.pose.position.y =
      m_pose.y;  //(tileY + 0.5 - ORIGIN_Y) * m_tile_resolution;
  startPose.pose.position.z = 0.0;

  q.setRPY(0, 0, m_pose.psi);
  startPose.pose.orientation.x = q.x();
  startPose.pose.orientation.y = q.y();
  startPose.pose.orientation.z = q.z();
  startPose.pose.orientation.w = q.w();

  otter_coverage::DubinInput di;
  di.header.stamp = ros::Time::now();
  di.header.frame_id = "map";
  di.start = startPose;
  di.end = goalPose;
  m_dubinPub.publish(di);
  */
}

}  // namespace otter_coverage
