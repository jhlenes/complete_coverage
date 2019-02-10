#ifndef PARTITION_BINN_H_
#define PARTITION_BINN_H_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>

class PartitionBinn {
 public:
  enum CellStatus { Unknown, Free, Blocked };

  struct Cell {
    Cell() : isCovered(false), status(Unknown) {}
    bool isCovered;
    CellStatus status;
  };

  PartitionBinn();

  void initialize(double x0, double y0, double x1, double y1, double rc);

  void update(const nav_msgs::OccupancyGrid& map, double x, double y);

  CellStatus calculateStatus(const nav_msgs::OccupancyGrid& map, int l, int k);

  void gridToWorld(int l, int k, double& xc, double& yc);

  void worldToGrid(double xc, double yc, int& l, int& k);

 private:
  /**
   * @brief given the column l and the row k, calculates
   * the center coordinates of the circle [xc, yc].
   * @param l the column 1<=l<=m
   * @param k the row 1<=k<=n[l]
   * @param xc
   * @param yc
   */
  void gridToLocal(int l, int k, double& xc, double& yc);

  /**
   * @brief given the cartesian coordinates [xc, yc], calculates the column l
   * and row k in the grid
   * @param xc
   * @param yc
   * @param l the column 1<=l<=m
   * @param k the row 1<=k<=n[l]
   */
  void localToGrid(double xc, double yc, int& l, int& k);

  bool m_initialized;

  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;
  double m_rc;
  double m_Xw;
  double m_Yw;

  int m_m;               // columns
  std::vector<int> m_n;  // rows

  std::vector<std::vector<Cell>> m_cells;  // m_cells[column][row]
};

#endif
