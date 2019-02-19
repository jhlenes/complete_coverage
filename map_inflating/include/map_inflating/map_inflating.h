#ifndef OTTER_MAP_INFLATING_H_
#define OTTER_MAP_INFLATING_H_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <vector>

class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(unsigned int i, unsigned int x, unsigned int y, unsigned int sx,
           unsigned int sy)
      : index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

class MapProcessor
{

public:
  MapProcessor();
  ~MapProcessor();

private:
  // ROS params
  double m_inflation_radius;

  std::vector<CellData> inflation_cells_;
  ros::Publisher publisher;

  void processMap(const nav_msgs::OccupancyGrid& grid);
  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y, bool seen[],
                      double resolution);
};

#endif
