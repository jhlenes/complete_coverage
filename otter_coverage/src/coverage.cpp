#include <coverage/coverage.h>

#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

#include <queue>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

namespace otter_coverage {

    Coverage::Coverage() {

        ros::NodeHandle nh;

        ros::Subscriber sub = nh.subscribe("inflated_map", 1000, &Coverage::mapCallback, this);

        m_goalPub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
        m_pathPub = nh.advertise<nav_msgs::Path>("covered_path", 1000);

        mainLoop(nh);
    }

    Coverage::~Coverage() {

    }

    void Coverage::mapCallback(const nav_msgs::OccupancyGrid &grid) {
        if (!m_mapInitialized) {
            m_mapInitialized = true;
        }
        m_grid = grid;
    }

    void Coverage::mainLoop(ros::NodeHandle nh)
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Rate rate(10.0);
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

        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0), ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex) {
            return false;
        }

        m_pose.x = transformStamped.transform.translation.x;
        m_pose.y = transformStamped.transform.translation.y;
        tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                         transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        m_pose.psi = tf::getYaw(q);

        return true;
    }

    void Coverage::boustrophedonMotion() {

        // we need to wait until we have a map
        if (!m_mapInitialized) {
            return;
        }

        // find tile where robot is located
        int tileX = std::floor(m_pose.x / TILE_RESOLUTION) + ORIGIN_X;
        int tileY = std::floor(m_pose.y / TILE_RESOLUTION) + ORIGIN_Y;

        // TODO: Another data structure?
        if (tileX < 0 || tileX > TILE_SIZE - 1 || tileY < 0 || tileY > TILE_SIZE - 1) {
            ROS_WARN_STREAM("Tile map is too small (0 - " << TILE_SIZE << "). x: " << tileX << " y: " << tileY);
        }

        static Goal goal = {true, true, tileX, tileY};

        // add starting point
        static bool initialized = false;
        if (!initialized) {
            publishGoal(tileY, tileX, goal);
            initialized = true;
        }

        // we want to go back to the start once we are finished
        static const int startX = tileX;
        static const int startY = tileY;
        static bool finished = false;

        checkGoal(tileX, tileY, goal);

        // check to find the first available direction in the priority of north-south-east-west.
        checkDirection(1, 0, tileX, tileY, goal);
        checkDirection(-1, 0, tileX, tileY, goal);
        checkDirection(0, -1, tileX, tileY, goal);
        checkDirection(0, 1, tileX, tileY, goal);

        // a new goal has been given manually
        if (finished && goal.isNew) {
            finished = false;
        }

        // if all directions are blocked, then the critical point has been reached
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

    void Coverage::checkGoal(int tileX, int tileY, Goal &goal)
    {
        // check if the robot is within a circle of acceptance with radius GOAL_TOLERANCE
        if (goal.exists) {
            geometry_msgs::PoseStamped goalPose = m_coveredPath.poses[m_coveredPath.poses.size() - 1];
            if (std::sqrt(std::pow(goalPose.pose.position.x - m_pose.x, 2) + std::pow(goalPose.pose.position.y - m_pose.y, 2)) < GOAL_TOLERANCE) {
                goal.exists = false;
                goal.isNew = false;
                M[tileX][tileY] = COVERED;
            }
        }
    }

    bool Coverage::isBacktrackingPoint(int i, int j) {

        // b(s1,s8) or b(s1,s2)
        bool eastBP = M[i][j-1] == FREE && (M[i+1][j-1] == BLOCKED || M[i-1][j-1] == BLOCKED);

        // b(s5,s6) or b(s5,s4)
        bool westBP = M[i][j+1] == FREE && (M[i+1][j+1] == BLOCKED || M[i-1][j+1] == BLOCKED);

        // b(s7,s6) or b(s7,s8)
        bool southBP = M[i-1][j] == FREE && (M[i-1][j+1] == BLOCKED || M[i-1][j-1] == BLOCKED);

        // Note: north can not be a BP because of the north-south-east-west check priority.

        return eastBP || westBP || southBP;
    }

    bool Coverage::locateBestBacktrackingPoint(int &goalX, int &goalY, int tileX, int tileY) {

        std::vector<tile> BPs;
        int closestPoint = 0;
        double minDistance = -1.0;

        for (int i = 1; i < TILE_SIZE-1; i++) {
            for (int j = 1; j < TILE_SIZE-1; j++) {
                if (M[i][j] != COVERED) {
                    continue;
                }
                if (isBacktrackingPoint(i, j)) {
                    tile point = {i, j};
                    BPs.push_back(point);

                    // check if closest point
                    double dist = std::sqrt(std::pow(i - tileX, 2) + std::pow(j - tileY, 2));
                    if (dist < minDistance || minDistance < 0) {
                        closestPoint = BPs.size() - 1;
                        minDistance = dist;
                    }
                }
            }
        }

        // find the nearest point
        if (BPs.size() > 0) {
            tile best = BPs.at(closestPoint);
            goalX = best.x;
            goalY = best.y;
            return true;
        }

        // no good backtracking point found, find any free point instead
        for (int i = 1; i < TILE_SIZE-1; i++) {
            for (int j = 1; j < TILE_SIZE-1; j++) {
                if (M[i][j] != COVERED) {
                    continue;
                }
                if (M[i][j-1] == FREE || M[i][j+1] == FREE || M[i-1][j] == FREE) {
                    goalX = i;
                    goalY = j;
                    return true;
                }
            }
        }

        // no free points, check whole map
        for (int i = 1; i < TILE_SIZE-1; i++) {
            for (int j = 1; j < TILE_SIZE-1; j++) {
                if (isFree(i, j, false)) {
                    goalX = i;
                    goalY = j;
                    return true;
                }
            }
        }

        return false;
    }

    bool Coverage::isFree(int xTile, int yTile, bool allowUnknown) {

        // a tile contains many grid cells, need to check if all are free

        // find position of tile in map frame
        double xPos = (xTile - ORIGIN_X) * TILE_RESOLUTION;
        double yPos = (yTile - ORIGIN_Y) * TILE_RESOLUTION;

        // use map frame position to find corner grid cell in the tile
        int gridX = (xPos - m_grid.info.origin.position.x) / m_grid.info.resolution;
        int gridY = (yPos - m_grid.info.origin.position.y) / m_grid.info.resolution;

        // iterate through all grid cells in the tile, starting from the corner grid cell
        for (int i = 0; i * m_grid.info.resolution < TILE_RESOLUTION; i++) {
            for (int j = 0; j * m_grid.info.resolution < TILE_RESOLUTION; j++) {
                int gridIndex = (gridY+j)*m_grid.info.width+(gridX+i);
                if (m_grid.data[gridIndex] > 50) {
                    if (allowUnknown) {
                        return false;
                    } else if (m_grid.data[gridIndex] < 0) {
                        return false;
                    }

                }
            }
        }

        return true;
    }

    void Coverage::checkDirection(int xOffset, int yOffset, int tileX, int tileY, Goal &goal) {
        if (M[tileX + xOffset][tileY + yOffset] != COVERED) {
            if (isFree(tileX + xOffset, tileY + yOffset, true)) {

                M[tileX + xOffset][tileY + yOffset] = FREE;

                if (!goal.exists) {
                    ROS_INFO_STREAM("Moving to +x: " << xOffset << " +y: " << yOffset);
                    goal.x = tileX + xOffset;
                    goal.y = tileY + yOffset;
                    goal.exists = true;
                    goal.isNew = true;
                }
            } else {
                M[tileX + xOffset][tileY + yOffset] = BLOCKED;
            }
        }
    }

    void Coverage::publishGoal(int tileY, int tileX, Goal goal)
    {
        geometry_msgs::PoseStamped goalPose;

        goalPose.header.stamp = ros::Time::now();
        goalPose.header.frame_id = "map";

        // set goal to middle of tile
        goalPose.pose.position.x = (goal.x + 0.5 - ORIGIN_X) * TILE_RESOLUTION;
        goalPose.pose.position.y = (goal.y + 0.5 - ORIGIN_Y) * TILE_RESOLUTION;
        goalPose.pose.position.z = 0.0;

        double psi = std::atan2(goal.y-tileY, goal.x-tileX);
        tf::Quaternion q = tf::createQuaternionFromYaw(psi);

        goalPose.pose.orientation.x = q.x();
        goalPose.pose.orientation.y = q.y();
        goalPose.pose.orientation.z = q.z();
        goalPose.pose.orientation.w = q.w();

        m_goalPub.publish(goalPose);

        m_coveredPath.header.stamp = ros::Time::now();
        m_coveredPath.header.frame_id = "map";
        m_coveredPath.poses.push_back(goalPose);

        m_pathPub.publish(m_coveredPath);
    }


}
