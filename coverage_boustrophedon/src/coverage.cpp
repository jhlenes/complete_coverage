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
  m_x0 = nhP.param("x0", -10);
  m_y0 = nhP.param("y0", -51);
  m_x1 = nhP.param("x1", 40);
  m_y1 = nhP.param("y1", 50);
  m_tileResolution = nhP.param("tile_resolution", 1.0);
  m_scanRange = nhP.param("scan_range", 20.0);
  m_goalTolerance = nhP.param("goal_tolerance", 1.0);
  m_coverageSize = nhP.param("coverage_size", 5);

  // Set up partition
  m_partition.initialize(nh, m_x0, m_y0, m_x1, m_y1, m_tileResolution, m_scanRange);

  // Set up subscribers
  ros::Subscriber sub = nh.subscribe("inflated_map", 1000, &Coverage::mapCallback, this);

  // Set up publishers
  m_goalPub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  m_pathPub = nh.advertise<nav_msgs::Path>("covered_path", 1000);
  m_dubinPub = nh.advertise<coverage_boustrophedon::DubinInput>("simple_dubins_path/input", 1000);

  mainLoop(nh);
}

void Coverage::mapCallback(const nav_msgs::OccupancyGrid& grid)
{
  m_partition.update(grid, m_pose.x, m_pose.y);
  if (!m_mapInitialized)
    m_mapInitialized = true;
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

      // TODO: Get coverage size from depth
      // m_coverageSize = gy / (m_y1 - m_y0) * 5.5 + 2;
      if (m_coverageSize < m_minCoverageSize || m_minCoverageSize < 0) {
        m_minCoverageSize = m_coverageSize;
      }

      Goal goal = updateWPs(gx, gy);

      boustrophedonCoverage(gx, gy, goal);

      m_partition.setCovered(gx, gy, true, m_coverageSize, m_pose.psi);

      if (!m_dirInitialized)
      {
        m_dirInitialized = true;
        newTrack(gx, gy);
      }

      // ROS_INFO_STREAM("Current pos: " << gx << ", " << gy);
      // ROS_INFO_STREAM("Current goal: " << goal.gx << ", " << goal.gy);
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
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0), ros::Duration(1.0));
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
  if (goal.reached)
  {
    if (checkDirection(m_dir, goal.gx, goal.gy) && !m_backtracking)
    {
      ROS_INFO_STREAM("Moving to new cell...");
    }
    else
    {
      // A critical point has been reached
      ROS_INFO_STREAM("Critical point reached.");
      int bpX, bpY;
      std::vector<Tile> path;
      if (locateBestBacktrackingPoint(bpX, bpY, gx, gy, path))
      {
        ROS_INFO_STREAM("Backtracking to: x=" << bpX << ", y=" << bpY << " From: x=" << gx << ", y=" << gy);
        m_backtracking = true;
        path = aStarSPT(m_partition, {gx, gy}, {bpX, bpY});
        for (auto it = path.begin(); it != path.end(); it++)
        {
          static int bpCount = 1;
          ROS_INFO_STREAM("BP path counter: " << bpCount++);
          m_waypoints.push_back(*it);
        }
        ROS_INFO_STREAM("Backtracking waypoints added: " << int(path.size()));

        // Go further in m_sweepDir. Now only utilizes half of coverageSize.
        if (m_partition.withinGridBounds(bpX + 1, bpY) && freeAndNotCovered(bpX + 1, bpY))
          m_dir = North;
        else
          m_dir = South;

        if (m_partition.withinGridBounds(bpX, bpY - 1) && freeAndNotCovered(bpX, bpY - 1))
          m_sweepDir = East;
        else
          m_sweepDir = West;

        int yDir = (m_sweepDir == East ? -1 : 1);
        for (int y = bpY + yDir; std::abs(y - bpY) <= m_coverageSize - 1; y += yDir)
        {
          if (m_partition.withinGridBounds(bpX, y) && m_partition.getStatus(bpX, y) == Partition::Free)
          {
            m_waypoints.push_back({bpX, y});
          }
          else
          {
            newTrack(bpX, y - yDir);
            return;
          }
        }
        newTrack(m_waypoints.back().gx, m_waypoints.back().gy);
      }
      else
      {
        ROS_INFO("No backtracking point found!");
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
  static bool goalPublished = true;

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
      m_waypoints.clear(); // clear waypoints

      if (m_backtracking || m_finished)
        newTrack(gx, gy);

      if (m_finished)
        m_finished = false;
    }
  }

  // Get new goal from list of waypoints
  if (goal.reached && !m_waypoints.empty())
  {
    goal = Goal(m_waypoints.front());
    m_waypoints.pop_front();
    goalPublished = false;

    if (m_waypoints.empty() && m_backtracking) {
      m_backtracking = false;
    }
  }

  // Publish waypoint
  if (!goalPublished)
  {
    publishGoal(gx, gy, goal);
    goalPublished = true;
  }

  // Finished
  // TODO: Figure out how not to be "finished" when in starting position
  if ((gx != startX || gy != startY) && goal.reached && m_waypoints.empty() &&
      m_partition.hasCompleteCoverage() && !m_finished)
  {
    ROS_INFO("Finished!");
    m_finished = true;
    m_wallFollowing = false;

    // Go to start
    ROS_INFO("Going to start!");
    auto path = aStarSPT(m_partition, {gx, gy}, {startX, startY});
    for (auto it = path.begin(); it != path.end(); it++)
    {
      m_waypoints.push_back(*it);
    }
    goal = Goal(m_waypoints.front());
    m_waypoints.pop_front();
    goalPublished = false;    
  }

  if (m_finished && !m_partition.hasCompleteCoverage())
  {
    m_finished = false;
  }

  return goal;
}

bool Coverage::blockedOrCovered(int gx, int gy)
{
  if (!m_partition.withinGridBounds(gx, gy))
  {
    return true;
  }
  return m_partition.getStatus(gx, gy) != Partition::Free || m_partition.isCovered(gx, gy);
}

bool Coverage::freeAndNotCovered(int gx, int gy)
{
  if (!m_partition.withinGridBounds(gx, gy))
  {
    return false;
  }
  return m_partition.getStatus(gx, gy) == Partition::Free && !m_partition.isCovered(gx, gy);
}

bool Coverage::isBacktrackingPoint(int gx, int gy, Tile& bp)
{
  if (!m_partition.isCovered(gx, gy) || m_partition.getStatus(gx,gy) == Partition::Blocked)
  {
    return false;
  }

  // b(s1,s8) or b(s1,s2)
  bool eastBP = false;
  if (gy - 1 >= 0)
  {
    eastBP = freeAndNotCovered(gx, gy - 1) && (blockedOrCovered(gx + 1, gy - 1) || blockedOrCovered(gx - 1, gy - 1));
  }
  if (eastBP)
  {
    bp = {gx, gy - 1};
    return true;
  }

  // b(s5,s6) or b(s5,s4)
  bool westBP = false;
  if (gy + 1 < m_partition.getHeight())
  {
    westBP = freeAndNotCovered(gx, gy + 1) && (blockedOrCovered(gx + 1, gy + 1) || blockedOrCovered(gx - 1, gy + 1));
  }
  if (westBP)
  {
    bp = {gx, gy + 1};
    return true;
  }

  // b(s7,s6) or b(s7,s8)
  bool southBP = false;
  if (gx - 1 >= 0)
  {
    southBP = freeAndNotCovered(gx - 1, gy) && (blockedOrCovered(gx - 1, gy + 1) || blockedOrCovered(gx - 1, gy - 1));
  }
  if (southBP)
  {
    bp = {gx - 1, gy};
    return true;
  }

  bool northBP = false;
  if (gx + 1 < m_partition.getWidth())
  {
    northBP = freeAndNotCovered(gx + 1, gy) && (blockedOrCovered(gx + 1, gy + 1) || blockedOrCovered(gx + 1, gy - 1));
  }
  if (northBP)
  {
    bp = {gx + 1, gy};
    return true;
  }

  return false;
}

bool Coverage::locateBestBacktrackingPoint(int& goalX, int& goalY, int tileX, int tileY, std::vector<Tile>& bestPath)
{
  int minDistance = -1;
  for (int gx = 0; gx < m_partition.getWidth(); gx++)
  {
    for (int gy = 0; gy < m_partition.getHeight(); gy++)
    {
      Tile bp;
      if (isBacktrackingPoint(gx, gy, bp))
      {
        auto path = aStarSearch(m_partition, {tileX, tileY}, {gx, gy});
        if (path.size() == 0) {
          continue; // BP not reachable
        }
        path.push_back(bp);
        int distance = int(path.size());
        if (distance < minDistance || minDistance < 0)
        {
          minDistance = distance;
          bestPath = path;
          goalX = bp.gx;
          goalY = bp.gy;
        }
      }
    }
  }

  // TODO: search for any free reachable cell

  if (minDistance < 0)
    return false;

  return true;
}

bool Coverage::checkDirection(Direction dir, int gx, int gy)
{
  int xOffset = (dir == North ? 1 : -1);
  int yOffset = (m_sweepDir == West ? 1 : -1);

  /* Check first if we should continue moving straight forward */

  // Straight ahead is free?
  if (m_partition.withinGridBounds(gx + xOffset, gy) && m_partition.getStatus(gx + xOffset, gy) == Partition::Free)
  {
    // Any free uncovered cells in moving direction?
    for (int x = gx + xOffset; std::abs(x - gx) <= 2 * m_coverageSize; x += xOffset)
    {
      for (int j = -m_coverageSize; j <= m_coverageSize; j++)
      {
        int y = gy + j;
        if (m_partition.withinGridBounds(x, y) && freeAndNotCovered(x, y))
        {
          // Add goal
          m_wallFollowing = false;
          m_waypoints.push_back({gx + xOffset, gy});
          return true;
        }
      }
    }

    // Only covered cells ahead. Backtracking for now... TODO: smarter way to handle this?
    return false;
  }

  // Straight ahead is unknown? Continue forward, because most likely rolling window is not updated correctly.
  // TODO: smarter way to handle this?
  if (m_partition.withinGridBounds(gx + xOffset, gy) && m_partition.getStatus(gx + xOffset, gy) == Partition::Unknown)
  {
    // Add goal
    m_waypoints.push_back({gx + xOffset, gy});
    return true;
  }

  /* At this point we know that straight ahead is blocked. We should consider
  wall following to either side if there are free cells adjacent to the current track. */

  ROS_INFO_STREAM("Straight ahead is blocked.");
  ROS_INFO_STREAM("Current track: x=" << m_trackX << ", y=" << m_trackY);

  // Check if free cells in sweep direction
  static int wallFollowDist = 0;
  if (!m_wallFollowing)
  {
    wallFollowDist = m_minCoverageSize;

    // Check if there are target cells adjacent to current track in sweep direction
    ROS_INFO_STREAM("Checking whether or not to wall follow...");
    for (int y = m_trackY + yOffset; std::abs(y - m_trackY) <= m_coverageSize * 2; y += yOffset)
    {
      for (int x = gx + xOffset; x != m_trackX; x -= xOffset)
      {
        if (freeAndNotCovered(x, y))
        {
          m_wallFollowing = true;
          goto endCheckWallFollow;
        }
      }
    }

    // Check opposite sweep direction
    for (int y = m_trackY + yOffset; std::abs(y - m_trackY) <= m_coverageSize * 2; y -= yOffset)
    {
      for (int x = gx + xOffset; x != m_trackX; x -= xOffset)
      {
        if (freeAndNotCovered(x, y))
        {
          // Switch sweep direction
          m_sweepDir = (m_sweepDir == East ? West : East);
          yOffset *= -1;
          ROS_INFO_STREAM("Switching sweep direction to: " << (m_sweepDir == East ? "East" : "West"));
          
          m_wallFollowing = true;
          goto endCheckWallFollow;
        }
      }
    }
  }
endCheckWallFollow:

  if (m_wallFollowing)
  {
    // Wall follow in sweep direction at maximum 2*m_coverageSize cells
    int nextX = gx + xOffset;
    for (int y = gy + yOffset; std::abs(y - m_trackY) != wallFollowDist * 2 + 1; /* 1 cell overlap */ y += yOffset)
    {
      for (int x = nextX; std::abs(x - nextX) <= 2 * m_coverageSize; x -= xOffset)
      {
        if (m_partition.withinGridBounds(x, y) && m_partition.getStatus(x, y) == Partition::Free)
        {
          // Add goal
          m_waypoints.push_back({x, y});
          ROS_INFO_STREAM("Wall following to: " << x << ", " << y << " from: " << gx << ", " << gy);
          return true;
        }
      }
      // TODO: Check if finished? x - nextX == 2 * m_coverageSize 
      // TODO: If here, wall following failed? Can't move through blocked wall..
      // return false;
    }


    // We are finished wall following, switch direction and start new track
    m_dir = (m_dir == North ? South : North);
    newTrack(gx, gy);
    ROS_INFO_STREAM("Switching direction to " << (m_dir == North ? "North" : "South"));
    m_wallFollowing = false;
    return true;
  }

  /* At this point we know that straight forward is blocked, and that we should not wall follow.
  Thus, the boustrophedon motion is finished, and we should backtrack */
  ROS_INFO_STREAM("Boustrophedon motion finished.");
  return false;
}

void Coverage::publishGoal(int gx, int gy, Goal goal)
{
  //static SimpleDubinsPath dubin(m_tileResolution / 2.0, 0.1);

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

void Coverage::newTrack(int gx, int gy) {
  m_trackX = gx;
  m_trackY = gy;
  m_minCoverageSize = -1;
}

} // namespace otter_coverage
