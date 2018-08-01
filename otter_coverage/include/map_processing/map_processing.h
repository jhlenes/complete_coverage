#ifndef OTTER_MAP_PROCESSING_H_
#define OTTER_MAP_PROCESSING_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace otter_coverage {

    class MapProcessor {
        public:
            MapProcessor();
            ~MapProcessor();
        private:
            void processMap(const nav_msgs::OccupancyGrid &grid);
    };

}

#endif
