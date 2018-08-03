#include <map_processing/map_processing.h>

#include <algorithm>
#include <math.h>

namespace otter_coverage {

    MapProcessor::MapProcessor() {

        ros::NodeHandle nh;

        // subscribe to occupancy grid map from slam node
        ros::Subscriber sub = nh.subscribe("map", 1000, &MapProcessor::processMap, this);

        publisher = nh.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1000);

        ros::spin();
    }

    MapProcessor::~MapProcessor() {

    }

    void MapProcessor::processMap(const nav_msgs::OccupancyGrid &grid) {
        // Inflate the obstacles of the map. I.e. iterate through the grid and expand the obstacles to nearby cells.

        inflation_cells_.clear();

        nav_msgs::OccupancyGrid newGrid = grid;

        // keep track of the cells we have seen
        size_t gridSize = grid.info.height * grid.info.width;
        bool seen[gridSize];
        std::fill_n(seen, gridSize, false);

        for (int x = 0; x < grid.info.height; x++) {
            for (int y = 0; y < grid.info.width; y++) {
                int gridIndex = x * grid.info.width + y;

                // apply threshold function and locate all obstacles
                if (grid.data[gridIndex] > 50) {
                    newGrid.data[gridIndex] = 100;
                    inflation_cells_.push_back(CellData(gridIndex, x, y, x, y));
                } else if (grid.data[gridIndex] >= 0) {
                    newGrid.data[gridIndex] = 0;
                }
            }
        }

        // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
        // can overtake previously inserted but farther away cells
        for (int i = 0; i < inflation_cells_.size(); i++) {

            // process all cells at distance dist_bin.first
            CellData cell = inflation_cells_[i];

            unsigned int index = cell.index_;

            // ignore if already visited
            if (seen[index]) {
                continue;
            }

            seen[index] = true;

            newGrid.data[index] = 100;

            unsigned int mx = cell.x_;
            unsigned int my = cell.y_;
            unsigned int sx = cell.src_x_;
            unsigned int sy = cell.src_y_;

            // attempt to put the neighbors of the current cell onto the inflation list
            double res = grid.info.resolution;
            if (mx > 0) {
                enqueue(index - grid.info.width, mx - 1, my, sx, sy, seen, res);
            }
            if (my > 0){
                enqueue(index - 1, mx, my - 1, sx, sy, seen, res);
            }
            if (mx < grid.info.height - 1){
                enqueue(index + grid.info.width, mx + 1, my, sx, sy, seen, res);
            }
            if (my < grid.info.width - 1){
                enqueue(index + 1, mx, my + 1, sx, sy, seen, res);
            }
        }

        publisher.publish(newGrid);

    }

    /**
     * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
     * @param  index The index of the cell
     * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
     * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
     * @param  src_x The x index of the obstacle point inflation started at
     * @param  src_y The y index of the obstacle point inflation started at
     */
    inline void MapProcessor::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                        unsigned int src_x, unsigned int src_y, bool seen[], double resolution)
    {
        if (!seen[index]) {
            double distance = std::sqrt(std::pow((double)mx - (double)src_x, 2) + std::pow((double)my - (double)src_y, 2));
            if (distance*resolution <= INFLATED_RADIUS) {
                // push the cell data onto the inflation list
                inflation_cells_.push_back(CellData(index, mx, my, src_x, src_y));
            }
        }
    }

}
