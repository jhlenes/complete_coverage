#include <coverage/coverage.h>

#include <coverage_boustrophedon/DubinInput.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <math.h>
#include <queue>
#include <vector>

namespace otter_coverage
{

Coverage::Coverage()
{
  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");

  // Get parameters
  m_x0 = nhP.param("x0", -51);
  m_y0 = nhP.param("y0", -51);
  m_x1 = nhP.param("x1", 50);
  m_y1 = nhP.param("y1", 50);
  m_tileResolution = nhP.param("tile_resolution", 5.0);
  m_scanRange = nhP.param("scan_range", 24.5);
  m_goalTolerance = nhP.param("goal_tolerance", 1.0);

  // Set up partition
  m_partition.initialize(nh, m_x0, m_y0, m_x1, m_y1, m_tileResolution,
                         m_scanRange);

  // TODO: remove
  m_coverageSize = 5;

  // Set up subscribers
  ros::Subscriber sub =
      nh.subscribe("inflated_map", 1000, &Coverage::mapCallback, this);

  // Set up publishers
  m_goalPub =
      nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  m_pathPub = nh.advertise<nav_msgs::Path>("covered_path", 1000);
  m_dubinPub = nh.advertise<coverage_boustrophedon::DubinInput>(
      "simple_dubins_path/input", 1000);

  mainLoop(nh);
}

void Coverage::mapCallback(const nav_msgs::OccupancyGrid& grid)
{
  m_partition.update(grid, m_pose.x, m_pose.y);
  if (!m_mapInitialized)
  {
    m_mapInitialized = true;
  }
}

void Coverage::mainLoop(ros::NodeHandle nh)
{
  ros::Rate rate(5.0);
  while (nh.ok())
  {
    if (!updatePose())
    {
      ROS_WARN("Transform from map to base_link not found.");
      ros::Duration(1.0).sleep();
      continue;
    }

    if (m_mapInitialized)
    {
      // Find grid cell where robot is located
      int gx, gy;
      m_partition.worldToGrid(m_pose.x, m_pose.y, gx, gy);

      Goal goal = updateWPs(gx, gy);

      boustrophedonCoverage(gx, gy, goal);
    }

    ros::spinOnce();
    rate.sleep();
  }
}

bool Coverage::updatePose()
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped tfStamped;
  try
  {
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                         ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  m_pose.x = tfStamped.transform.translation.x;
  m_pose.y = tfStamped.transform.translation.y;
  m_pose.psi = tf2::getYaw(tfStamped.transform.rotation);

  return true;
}

void Coverage::boustrophedonCoverage(int gx, int gy, Goal goal)
{
  const auto priorities = m_priorities;
  if (goal.reached)
  {
    // Check to find the next target in north-south-east-west priority.
    int newX;
    int newY;
    if (checkDirection(priorities[0], goal.gx, goal.gy, newX, newY))
    {
      m_waypoints.push_back({newX, newY});
    }
    else if (checkDirection(priorities[1], goal.gx, goal.gy, newX, newY))
    {
      m_waypoints.push_back({newX, newY});
    }
    else if (checkDirection(priorities[2], goal.gx, goal.gy, newX, newY))
    {
      m_waypoints.push_back({newX, newY});
    }
    else if (checkDirection(priorities[3], goal.gx, goal.gy, newX, newY))
    {
      m_waypoints.push_back({newX, newY});
    }
    else
    {
      // If all directions are blocked, then a critical point has been reached
      int bpX, bpY;
      if (locateBestBacktrackingPoint(bpX, bpY, gx, gy))
      {
        ROS_INFO("Critical point! Backtracking...");
        auto path = aStarSearch(m_partition, {gx, gy}, {bpX, bpY});
        for (auto it = path.begin() + 1; it != path.end(); it++)
        {
          m_waypoints.push_back(*it);
        }
      }
      else
      {
        ROS_INFO("No critical point found!");
      }
    }
  }
}

Coverage::Goal Coverage::updateWPs(int gx, int gy)
{
  static const int startX = gx;
  static const int startY = gy;
  static bool initialized = false;
  static Goal goal({startX, startY});
  static bool goalPublished = false;

  // Set initial grid position to covered
  if (!initialized)
  {
    // m_partition.setCovered(startX, startY, true);
    goal.reached = true;
    initialized = true;
  }

  // Has active waypoint
  if (!goal.reached)
  {
    // Check if waypoint is reached
    double goalX, goalY;
    m_partition.gridToWorld(goal.gx, goal.gy, goalX, goalY);
    if (m_partition.dist(goalX, goalY, m_pose.x, m_pose.y) < m_goalTolerance)
    {
      m_partition.setCovered(goal.gx, goal.gy, true, m_coverageSize);
      goal.reached = true;
    }

    // Check if waypoint is blocked
    if (m_partition.getStatus(goal.gx, goal.gy) == Partition::Blocked)
    {
      ROS_INFO("Current waypoint is blocked. Searching for new... ");
      goal.reached = true;
    }
  }

  // Get new goal from list of waypoints
  if (goal.reached && !m_waypoints.empty())
  {
    goal = Goal(m_waypoints.front());
    m_waypoints.pop_front();
    goalPublished = false;
  }

  // Publish waypoint
  if (!goalPublished)
  {
    publishGoal(gx, gy, goal);
    goalPublished = true;
  }

  // Finished
  static bool finished = false;
  // TODO: Figure out how not to be "finished" when in starting position
  if ((gx != startX || gy != startY) && goal.reached && m_waypoints.empty() &&
      m_partition.hasCompleteCoverage() && !finished)
  {
    ROS_INFO("Finished!");
    finished = true;
    static bool goingToStart = false;

    // Go to start
    if (!goingToStart)
    {
      ROS_INFO("Going to start!");
      auto path = aStarSearch(m_partition, {gx, gy}, {startX, startY});
      for (auto it = path.begin() + 1; it != path.end(); it++)
      {
        m_waypoints.push_back(*it);
      }
      goal = Goal(m_waypoints.front());
      m_waypoints.pop_front();
      goalPublished = false;
      goingToStart = true;
    }
  }

  if (finished && !m_partition.hasCompleteCoverage())
  {
    finished = false;
  }

  return goal;
}

bool Coverage::blockedOrCovered(int gx, int gy)
{
  if (!m_partition.withinGridBounds(gx, gy))
  {
    return true;
  }
  return m_partition.getStatus(gx, gy) != Partition::Free ||
         m_partition.isCovered(gx, gy);
}

bool Coverage::freeAndNotCovered(int gx, int gy)
{
  if (!m_partition.withinGridBounds(gx, gy))
  {
    return false;
  }
  return m_partition.getStatus(gx, gy) == Partition::Free &&
         !m_partition.isCovered(gx, gy);
}

bool Coverage::isBacktrackingPoint(int gx, int gy)
{
  if (!m_partition.isCovered(gx, gy))
  {
    return false;
  }

  // b(s1,s8) or b(s1,s2)
  bool eastBP = false;
  if (gy - 1 >= 0)
  {
    eastBP =
        freeAndNotCovered(gx, gy - 1) &&
        (blockedOrCovered(gx + 1, gy - 1) || blockedOrCovered(gx - 1, gy - 1));
  }

  // b(s5,s6) or b(s5,s4)
  bool westBP = false;
  if (gy + 1 < m_partition.getHeight())
  {
    westBP =
        freeAndNotCovered(gx, gy + 1) &&
        (blockedOrCovered(gx + 1, gy + 1) || blockedOrCovered(gx - 1, gy + 1));
  }

  // b(s7,s6) or b(s7,s8)
  bool southBP = false;
  if (gx - 1 >= 0)
  {
    southBP =
        freeAndNotCovered(gx - 1, gy) &&
        (blockedOrCovered(gx - 1, gy + 1) || blockedOrCovered(gx - 1, gy - 1));
  }
  // Note: north can not be a BP because of the north-south-east-west check
  // priority.

  return eastBP || westBP || southBP;
}

bool Coverage::locateBestBacktrackingPoint(int& goalX, int& goalY, int tileX,
                                           int tileY)
{
  int minDistance = -1;
  for (int gx = 0; gx < m_partition.getWidth(); gx++)
  {
    for (int gy = 0; gy < m_partition.getHeight(); gy++)
    {
      if (isBacktrackingPoint(gx, gy))
      {
        // TODO: change to A*SPT distance
        int distance =
            int(aStarSearch(m_partition, {tileX, tileY}, {gx, gy}).size());
        if (distance < minDistance || minDistance < 0)
        {
          minDistance = distance;
          goalX = gx;
          goalY = gy;
        }
      }
    }
  }

  // TODO: search for any free reachable cell

  if (minDistance < 0)
    return false;

  return true;
}

bool Coverage::checkDirection(Direction dir, int gx, int gy, int& newX,
                              int& newY)
{
  // Check along the supplied direction until free or blocked space is
  // discovered

  int xOffset, yOffset;
  switch (dir)
  {
  case North:
    xOffset = 1;
    yOffset = 0;
    break;
  case South:
    xOffset = -1;
    yOffset = 0;
    break;
  case East:
    xOffset = 0;
    yOffset = -1;
    break;
  case West:
    xOffset = 0;
    yOffset = 1;
    break;
  }

  // Iterate along direction
  newX = gx;
  newY = gy;
  while (m_partition.withinGridBounds(newX + xOffset, newY + yOffset))
  {
    newX += xOffset;
    newY += yOffset;
    if (std::abs(newX - gx) > 2 * m_coverageSize ||
        std::abs(newY - gy) > 2 * m_coverageSize)
    {
      break;
    }

    // Straight ahead is blocked? Wall follow one cell to either direction
    // TODO: More robust wall following
    if (m_partition.getStatus(newX, newY) == Partition::Blocked)
    {
      if (m_partition.withinGridBounds(newX + yOffset, newY + xOffset) &&
          m_partition.getStatus(newX + yOffset, newY + xOffset) ==
              Partition::Free)
      {
        newX += yOffset;
        newY += xOffset;
      }
      else if (m_partition.withinGridBounds(newX - yOffset, newY - xOffset) &&
               m_partition.getStatus(newX - yOffset, newY - xOffset) ==
                   Partition::Free)
      {
        newX -= yOffset;
        newY -= xOffset;
      }
      else
      {
        // This direction is blocked
        return false;
      }

      // When movement in perpendicular direction is greater than
      // coverageSize, stop wall following
      if (((m_dir == North || m_dir == South) &&
           std::abs(newY - m_trackY) > 2 * m_coverageSize) ||
          ((m_dir == East || m_dir == West) &&
           std::abs(newX - m_trackX) > 2 * m_coverageSize))
      {
        return false;
      }
    }

    // Check for uncovered cells perpendicular to direction
    for (int j = -m_coverageSize; j <= m_coverageSize; j++)
    {
      int x2 = newX + yOffset * j;
      int y2 = newY + xOffset * j;
      if (m_partition.withinGridBounds(x2, y2) && freeAndNotCovered(x2, y2))
      {
        if (dir != m_dir)
        {
          if (dir == South)
          {
            m_priorities[0] = South;
            m_priorities[1] = North;
          }
          else
          {
            m_priorities[0] = North;
            m_priorities[1] = South;
          }
          if (dir == West)
          {
            m_priorities[2] = West;
            m_priorities[3] = East;
          }
          else
          {
            m_priorities[2] = East;
            m_priorities[3] = West;
          }

          m_lastDir = m_dir;
          m_dir = dir;
          m_trackX = gx;
          m_trackY = gy;

          ROS_INFO_STREAM("Moving: " << dir);
        }
        return true;
      }
    }
  }

  return false;
}

void Coverage::publishGoal(int gx, int gy, Goal goal)
{
  static SimpleDubinsPath dubin(m_tileResolution / 2.0, 0.1);

  geometry_msgs::PoseStamped startPose;
  startPose.header.stamp = ros::Time::now();
  startPose.header.frame_id = "map";
  startPose.pose.position.x = m_pose.x;
  startPose.pose.position.y = m_pose.y;
  startPose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, m_pose.psi);
  startPose.pose.orientation.x = q.x();
  startPose.pose.orientation.y = q.y();
  startPose.pose.orientation.z = q.z();
  startPose.pose.orientation.w = q.w();

  // Publish goal
  geometry_msgs::PoseStamped goalPose;
  goalPose.header.stamp = ros::Time::now();
  goalPose.header.frame_id = "map";
  double x, y;
  m_partition.gridToWorld(goal.gx, goal.gy, x, y);
  // double yaw;
  // dubin.getTargetHeading(m_pose.x, m_pose.y, m_pose.psi, x, y, yaw);
  goalPose.pose.position.x = x;
  goalPose.pose.position.y = y;
  goalPose.pose.position.z = 0.0;
  double yaw =
      std::atan2(y - startPose.pose.position.y, x - startPose.pose.position.x);
  q.setRPY(0, 0, yaw);
  goalPose.pose.orientation.x = q.x();
  goalPose.pose.orientation.y = q.y();
  goalPose.pose.orientation.z = q.z();
  goalPose.pose.orientation.w = q.w();
  // m_goalPub.publish(goalPose);

  // Publish covered path
  m_coveredPath.header.stamp = ros::Time::now();
  m_coveredPath.header.frame_id = "map";
  m_coveredPath.poses.push_back(goalPose);
  m_pathPub.publish(m_coveredPath);

  // Publish DubinInput
  coverage_boustrophedon::DubinInput di;
  di.header.stamp = ros::Time::now();
  di.header.frame_id = "map";
  di.start = startPose;
  di.end = goalPose;
  m_dubinPub.publish(di);
}

} // namespace otter_coverage
