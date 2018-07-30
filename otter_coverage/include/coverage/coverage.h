#ifndef OTTER_COVERAGE_H_
#define OTTER_COVERAGE_H_

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

namespace otter_coverage {

    const int UNKNOWN = 0;
    const int FREE = 1;
    const int COVERED = 2;
    const int BLOCKED = 3;

    const int TILE_SIZE = 100;

    class Coverage {
        public:
            Coverage();
            ~Coverage();
        private:
            void mainLoop(ros::NodeHandle nh);
            void mapCallback(const nav_msgs::OccupancyGrid &grid);
            bool isBacktrackingPoint(int i, int j);
            void locateBestBacktrackingPoint(int &goalX, int &goalY, int tileX, int tileY);
            bool isFree(int xTile, int yTile);
            void BM();

            ros::Publisher goalPub;
            ros::Publisher coveredPathPub;

            nav_msgs::Path coveredPath;
            nav_msgs::OccupancyGrid grid;

            struct pose {
                double x;
                double y;
                double psi;
            };

            struct tile {
                int x;
                int y;
            };


            int M[TILE_SIZE][TILE_SIZE] = {};
            pose otter;

            std::vector<tile> BP;
            int originX = 50;
            int originY = 50;
            int tileX = 50;
            int tileY = 50;
            double tileResolution = 1.0;

    };

}

#endif
