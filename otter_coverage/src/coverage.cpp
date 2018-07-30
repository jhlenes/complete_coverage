#include <coverage/coverage.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <queue>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

namespace otter_coverage {

    Coverage::Coverage() {

        ROS_WARN_STREAM("Constructor!");

        ros::NodeHandle nh;

        // subscribe to occupancy grid map from slam node
        ros::Subscriber sub = nh.subscribe("map", 1000, &Coverage::mapCallback, this);

        // publish next goal to the navigation stack's goal
        this->goalPub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

        this->coveredPathPub = nh.advertise<nav_msgs::Path>("covered_path", 1000);

        // Set current tile to covered
        M[originX][originY] = COVERED;

        mainLoop(nh);
    }

    Coverage::~Coverage() {

    }

    void Coverage::mainLoop(ros::NodeHandle nh)
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Rate rate(10.0);
        while (nh.ok()) {

            // get the pose of the robot in the map frame
            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0), ros::Duration(0.0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("Transform from map to base_link not found.");
                continue;
            }
            this->otter.x = transformStamped.transform.translation.x;
            this->otter.y = transformStamped.transform.translation.y;
            tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                             transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            this->otter.psi = tf::getYaw(q);

            BM();

            ros::spinOnce();
            rate.sleep();
        }
    }

    void Coverage::mapCallback(const nav_msgs::OccupancyGrid &grid) {
        this->grid = grid;
        return;
    }


    bool Coverage::isBacktrackingPoint(int i, int j) {
        // b(s1,s8) or b(s1,s2)
        bool eastFree = M[i][j-1] == FREE && (M[i+1][j-1] >= COVERED || M[i-1][j-1] >= COVERED);

        // b(s5,s6) or b(s5,s4)
        bool westFree = M[i][j+1] == FREE && (M[i+1][j+1] >= COVERED || M[i-1][j+1] >= COVERED);

        // b(s7,s6) or b(s7,s8)
        bool southFree = M[i-1][j] == FREE && (M[i-1][j+1] >= COVERED || M[i-1][j-1] >= COVERED);

        return eastFree || westFree || southFree;
    }

    void Coverage::locateBestBacktrackingPoint(int &goalX, int &goalY, int tileX, int tileY) {

        // find points
        BP.clear();
        int closestPoint = 0;
        double minDistance = -1.0;
        for (int i = 1; i < TILE_SIZE-1; i++) {
            for (int j = 1; j < TILE_SIZE-1; j++) {
                if (M[i][j] != COVERED) {
                    continue;
                }
                if (isBacktrackingPoint(i, j)) {
                    tile point = {i, j};
                    BP.push_back(point);

                    double dist = std::sqrt(std::pow(i - tileX, 2) + std::pow(j - tileY, 2));
                    if (dist < minDistance || minDistance < 0) {
                        closestPoint = BP.size() - 1;
                        minDistance = dist;
                    }
                }
            }
        }

        // find the nearest point. TODO: change to shortest path on covered tiles
        if (BP.size() > 0) {
            tile best = BP.at(closestPoint);
            goalX = best.x;
            goalY = best.y;
        }
    }

    bool Coverage::isFree(int xTile, int yTile) {

        if (grid.info.resolution == 0) return false;

        // Find start position of tile
        double xPos = (xTile - originX) * tileResolution;
        double yPos = (yTile - originY) * tileResolution;

        // Find occupancy grid position of tile
        int gridX = (xPos - grid.info.origin.position.x) / grid.info.resolution;
        int gridY = (yPos - grid.info.origin.position.y) / grid.info.resolution;

        // Check if all grid cells in the tile are free
        for (int i = 0; i * grid.info.resolution < tileResolution; i++) {
            for (int j = 0; j * grid.info.resolution < tileResolution; j++) {
                int gridIndex = (gridY+j)*grid.info.width+(gridX+i);
                if (grid.data[gridIndex] > 10) {
                    return false;
                }
            }
        }

        return true;
    }

    void Coverage::BM() {
        /*
        Inputs: The robot’s configuration and the model M of the workspace

        Outputs: Updated version of the robot’s configuration and the model M of the workspace

        Step 1. Check to find the first available direction in the
        priority of north-south-east-west. If all directions are
        blocked, then the critical point has been reached. Break the
        loop.

        Step 2. Move one step along this direction.

        Step 3. Generate the tile s = (x, y, 2r), i.e., the size of the robot’s diameter at the robot’s position.

        Step 4. Add the tile s to the mode M. Go to Step 1.
        */

        // Find tile where robot is located
        tileX = std::floor(otter.x / tileResolution) + originX;
        tileY = std::floor(otter.y / tileResolution) + originY;


        // check if next tile is free for obstacles and not covered already
        static bool hasGoal = false;
        static int goalX = tileX;
        static int goalY = tileY;
        if (goalX == tileX && goalY == tileY) {
            hasGoal = false;
            M[tileX][tileY] = COVERED;
        }

        if (M[tileX+1][tileY] != COVERED) {
            if (isFree(tileX+1, tileY)) {
                if (!hasGoal) {
                    ROS_WARN_STREAM("Moving to north tile!");
                    goalX++;
                    hasGoal = true;
                } else {
                    M[tileX+1][tileY] = FREE;
                }
            } else {
                M[tileX+1][tileY] = BLOCKED;
            }
        }
        if (M[tileX-1][tileY] != COVERED) {
            if (isFree(tileX-1, tileY)) {
                if (!hasGoal) {
                    ROS_WARN_STREAM("Moving to south tile!");
                    goalX--;
                    hasGoal = true;
                } else {
                    M[tileX-1][tileY] = FREE;
                }
            } else {
                M[tileX-1][tileY] = BLOCKED;
            }
        }
        if (M[tileX][tileY-1] != COVERED) {
            if (isFree(tileX, tileY-1)) {
                if (!hasGoal) {
                    ROS_WARN_STREAM("Moving to east tile!");
                    goalY--;
                    hasGoal = true;
                } else {
                    M[tileX][tileY-1] = FREE;
                }
            } else {
                M[tileX][tileY-1] = BLOCKED;
            }
        }
        if (M[tileX][tileY+1] != COVERED) {
            if (isFree(tileX, tileY+1)) {
                if (!hasGoal) {
                    ROS_WARN_STREAM("Moving to west tile!");
                    goalY++;
                    hasGoal = true;
                } else {
                    M[tileX][tileY+1] = FREE;
                }
            } else {
                M[tileX][tileY+1] = BLOCKED;
            }
        }
        if (!hasGoal) {
            ROS_WARN_STREAM("Critical point!");
            locateBestBacktrackingPoint(goalX, goalY, tileX, tileY);
            hasGoal = true;
        }

        // publish goal tile and covered path
        geometry_msgs::PoseStamped goal;

        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = (goalX + 0.5 - originX) * tileResolution;
        goal.pose.position.y = (goalY + 0.5 - originY) * tileResolution;
        goal.pose.position.z = 0.0;

        double psi = std::atan2(goalY-tileY, goalX-tileX);
        tf::Quaternion q = tf::createQuaternionFromYaw(psi);

        goal.pose.orientation.x = q.x();
        goal.pose.orientation.y = q.y();
        goal.pose.orientation.z = q.z();
        goal.pose.orientation.w = q.w();

        this->goalPub.publish(goal);

        coveredPath.header.stamp = ros::Time::now();
        coveredPath.header.frame_id = "map";
        coveredPath.poses.push_back(goal);

        this->coveredPathPub.publish(coveredPath);
    }

}
