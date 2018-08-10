#ifndef OTTER_COVERAGE_H_
#define OTTER_COVERAGE_H_

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>


namespace otter_coverage {

    // tile states
    const int UNKNOWN = 0;
    const int FREE = 1;
    const int COVERED = 2;
    const int BLOCKED = 3;

    const int TILE_SIZE = 100;
    const int ORIGIN_X = TILE_SIZE / 2;
    const int ORIGIN_Y = TILE_SIZE / 2;

    class Coverage {
        public:
            Coverage();
            ~Coverage();

    private:

            // ROS parameters
            double m_tile_resolution;
            double m_goal_tolerance;

            struct pose {
                double x;
                double y;
                double psi;
            };

            struct Goal {
                bool exists;
                bool isNew;
                int x;
                int y;
            };

            struct tile {
                int x;
                int y;
            };

            bool m_mapInitialized;

            ros::Publisher m_goalPub;
            ros::Publisher m_pathPub;

            nav_msgs::Path m_coveredPath;
            nav_msgs::OccupancyGrid m_grid;

            pose m_pose;

            int M[TILE_SIZE][TILE_SIZE] = {};


            void mapCallback(const nav_msgs::OccupancyGrid &m_grid);
            void mainLoop(ros::NodeHandle nh);
            bool updatePose(const tf2_ros::Buffer &tfBuffer);
            void boustrophedonMotion();
            void checkGoal(Goal &goal);
            void checkDirection(int xOffset, int yOffset, int tileX, int tileY, Goal &goal);
            bool isFree(int xTile, int yTile, bool allowUnknown);
            bool locateBestBacktrackingPoint(int &goalX, int &goalY, int tileX, int tileY);
            bool isBacktrackingPoint(int i, int j);
            void publishGoal(int tileY, int tileX, Goal goal);

    };

}

#endif
