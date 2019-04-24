#ifndef PARTITION_H_
#define PARTITION_H_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>

namespace otter_coverage
{

class Partition
{
public:
  enum Status
  {
    Unknown,
    Free,
    Blocked
  };
  struct Cell
  {
    Cell() : isCovered(false), status(Unknown) {}
    bool isCovered;
    Status status;
  };
  struct Point
  {
    int gx;
    int gy;
  };
  Partition();
  void initialize(ros::NodeHandle nh, double x0, double y0, double x1,
                  double y1, double cellSize, double scanRange);
  void drawPartition(int gx, int gy);
  void update(const nav_msgs::OccupancyGrid& map, double x, double y);
  void getNeighbors(int gx, int gy, double dist, std::vector<Point>& neighbors) const;
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;
  void worldToGrid(double wx, double wy, int& gx, int& gy) const;
  Status getStatus(int gx, int gy) const;
  void setStatus(int gx, int gy, Status status);
  bool isCovered(int gx, int gy) const;
  void setCovered(int gx, int gy, bool isCovered, int coverageSize = 0, double psi = 0.0);
  int getWidth() const;
  int getHeight() const;
  bool withinGridBounds(int gx, int gy) const;
  bool withinWorldBounds(double wx, double wy) const;
  bool hasCompleteCoverage() const;
  double dist(double x0, double y0, double x1, double y1) const
  {
    return std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
  }

private:
  void gridToLocal(int gx, int gy, double& x, double& y) const;
  void localToGrid(double x, double y, int& gx, int& gy) const;
  Status calcStatus(const nav_msgs::OccupancyGrid& map, int gx, int gy) const;

  ros::NodeHandle m_nh;
  ros::Publisher m_pub;

  bool m_initialized;

  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;
  int m_width;
  int m_height;

  double m_cellSize;
  double m_scanRange;

  std::vector<std::vector<Cell>> m_grid; // m_cells[gx][gy]
};

} // namespace otter_coverage

#endif
