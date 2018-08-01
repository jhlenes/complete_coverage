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
    const double TILE_RESOLUTION = 1.0;
    const int ORIGIN_X = 50;
    const int ORIGIN_Y = 50;

    class Coverage {
        public:
            Coverage();
            ~Coverage();

        private:
            void mapCallback(const nav_msgs::OccupancyGrid &grid);
            void mainLoop(ros::NodeHandle nh);
            bool updatePose(const tf2_ros::Buffer &tfBuffer);
            void BM();
            bool isFree(int xTile, int yTile);
            bool locateBestBacktrackingPoint(int &goalX, int &goalY, int tileX, int tileY);
            bool isBacktrackingPoint(int i, int j);

            bool m_mapInitialized;

            ros::Publisher goalPub;
            ros::Publisher coveredPathPub;

            nav_msgs::Path coveredPath;
            nav_msgs::OccupancyGrid grid;

            struct pose {
                double x;
                double y;
                double psi;
            };

            pose m_pose;

            struct tile {
                int x;
                int y;
            };

            int M[TILE_SIZE][TILE_SIZE] = {};
    };

}

#endif
