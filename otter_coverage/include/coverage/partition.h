#ifndef PARTITION_H_
#define PARTITION_H_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>

namespace otter_coverage {

class Partition {
 public:
  enum Status { Unknown, Free, Blocked };
  struct Cell {
    Cell() : isCovered(false), status(Unknown) {}
    bool isCovered;
    Status status;
  };
  struct Point {
    int col;
    int row;
  };
  Partition();
  Partition(ros::NodeHandle nh);

  void initialize(double x0, double y0, double x1, double y1, double cellSize,
                  double scanRange);

  void drawPartition();

  void update(const nav_msgs::OccupancyGrid& map, double x, double y);

  void gridToWorld(int col, int row, double& x, double& y);

  void worldToGrid(double x, double y, int& col, int& row);

  Status getStatus(int col, int row) { return m_cells[col][row].status; }

  void setStatus(int col, int row, Status status) {
    m_cells[col][row].status = status;
  }

  bool isCovered(int col, int row) { return m_cells[col][row].isCovered; }

  void setCovered(int col, int row, bool isCovered) {
    m_cells[col][row].isCovered = isCovered;
  }

  void getNeighbors(int col, int row, double dist,
                    std::vector<Point>& neighbors);

  bool hasCompleteCoverage();

  int getNumColumns() const;

  int getNumRows() const;

 private:
  void gridToLocal(int col, int row, double& x, double& y);

  void localToGrid(double x, double y, int& col, int& row);

  Status calcStatus(const nav_msgs::OccupancyGrid& map, int col, int row);

  ros::NodeHandle m_nh;
  ros::Publisher m_pub;

  bool m_initialized;

  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;

  int m_numColumns;
  int m_numRows;
  double m_cellSize;
  double m_scanRange;

  std::vector<std::vector<Cell>> m_cells;  // m_cells[column][row]
};

}  // namespace otter_coverage
#endif
