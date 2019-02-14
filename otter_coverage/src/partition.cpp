#include <coverage/partition.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>

namespace otter_coverage {

Partition::Partition() {}

Partition::Partition(ros::NodeHandle nh) : m_initialized(false) {
  ROS_INFO("Partition constructed.");

  m_nh = nh;
  m_pub = m_nh.advertise<visualization_msgs::MarkerArray>("cell_partition", 1);
}

void Partition::initialize(double x0, double y0, double x1, double y1,
                           double cellSize, double scanRange) {
  // TODO: rotate coordinate system? so that rectangle doesn't need to be
  // aligned with x and y axis in map frame.

  m_initialized = true;

  m_x0 = x0;
  m_y0 = y0;
  m_x1 = x1;
  m_y1 = y1;
  m_cellSize = cellSize;
  m_scanRange = scanRange;

  m_numColumns = static_cast<int>(std::ceil((x1 - x0) / cellSize));
  m_numRows = static_cast<int>(std::ceil((y1 - y0) / cellSize));

  ROS_INFO("rows: %d cols: %d", m_numRows, m_numColumns);

  // Initialize cells
  std::vector<std::vector<Cell>> cells;
  for (int col = 0; col < m_numColumns; col++) {
    std::vector<Cell> column;
    for (int row = 0; row < m_numRows; row++) {
      Cell cell;
      column.push_back(cell);
    }
    cells.push_back(column);
  }
  m_cells = cells;
}

void Partition::drawPartition() {
  if (!m_initialized) return;
  visualization_msgs::MarkerArray ma;
  int id = 0;
  for (int col = 0; col < m_numColumns; col++) {
    for (int row = 0; row < m_numRows; row++) {
      double x, y;
      gridToWorld(col, row, x, y);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "shapes";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.scale.x = m_cellSize;
      marker.scale.y = m_cellSize;
      marker.scale.z = 0.01;

      if (m_cells[col][row].status == Blocked) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
      } else if (m_cells[col][row].status == Free) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
      }
      if (m_cells[col][row].status == Unknown) {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
      }
      if (m_cells[col][row].isCovered) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      }
      marker.color.a = 0.1f;

      marker.lifetime = ros::Duration(0.0);
      ma.markers.push_back(marker);
    }
  }
  m_pub.publish(ma);
}

void Partition::getNeighbors(int col, int row, double dist,
                             std::vector<Point> &neighbors) {
  int maxCellDistance = static_cast<int>(std::ceil(dist / (m_cellSize)));

  double x0, y0;
  gridToWorld(col, row, x0, y0);

  for (int i = -maxCellDistance; i <= maxCellDistance; i++) {
    for (int j = -maxCellDistance; j <= maxCellDistance; j++) {
      if (col + i < 0 || col + i >= m_numColumns || row + j < 0 ||
          row + j >= m_numRows) {
        continue;
      }
      double x1, y1;
      gridToWorld(col + i, row + j, x1, y1);
      if (std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2)) <= dist) {
        neighbors.push_back({col + i, row + j});
      }
    }
  }
}

// TODO: consider using costmap_2d from navigation to get smaller maps
void Partition::update(const nav_msgs::OccupancyGrid &map, double x, double y) {
  // Update the status of cells based on the map received in a region around the
  // position [x,y]

  drawPartition();

  // Current cell
  int col, row;
  worldToGrid(x, y, col, row);

  // Calculate status for all nearby cells
  std::vector<Point> neighbors;
  getNeighbors(col, row, m_scanRange, neighbors);
  for (auto nb : neighbors) {
    setStatus(nb.col, nb.row, calcStatus(map, nb.col, nb.row));
  }
}

unsigned int mapIndex(unsigned int x, unsigned int y, unsigned int width) {
  // OccupancyGrid uses row major order
  return y * width + x;
}

Partition::Status Partition::calcStatus(const nav_msgs::OccupancyGrid &map,
                                        int col, int row) {
  /* A cell in the partition contains many cells in the map. All
   * map cells in a partition cell are free => partition cell is free
   */

  double xOrigin = map.info.origin.position.x;
  double yOrigin = map.info.origin.position.y;
  double mapRes = static_cast<double>(map.info.resolution);

  // Find position of partition cell in map frame
  double xWorld, yWorld;
  gridToWorld(col, row, xWorld, yWorld);

  // Find x and y in map of lower right corner of surrounding square
  unsigned int xMap =
      static_cast<unsigned int>(((xWorld - m_cellSize / 2) - xOrigin) / mapRes);
  unsigned int yMap =
      static_cast<unsigned int>(((yWorld - m_cellSize / 2) - yOrigin) / mapRes);

  // Check if all map cells in the partition cell is free
  bool unknown = false;
  for (unsigned int i = 0; i * mapRes < m_cellSize; i++) {
    for (unsigned int j = 0; j * mapRes < m_cellSize; j++) {
      unsigned int m = mapIndex(xMap + i, yMap + j, map.info.width);

      // Map not big enough
      if (xMap + i >= map.info.width || yMap + j >= map.info.height) {
        // If partition cell also contains blocked cells, the status should be
        // blocked. Therefore, flag.
        unknown = true;
      }

      // Any map cell with occupancy probability > 50
      if (map.data[m] > 50) {
        return Blocked;
      }

      // Assuming free if unknown. Remove comment to set cell to unknown
      // instead.
      // Any unknown map cell if (map.data[m] < 0) {
      //  unknown = true;
      //}
    }
  }

  // No blocked cells, but some are unknown.
  if (unknown) return Unknown;

  return Free;
}

int Partition::getNumRows() const { return m_numRows; }

int Partition::getNumColumns() const { return m_numColumns; }

void Partition::gridToWorld(int col, int row, double &x, double &y) {
  double xLocal;
  double yLocal;
  gridToLocal(col, row, xLocal, yLocal);
  x = xLocal + m_x0;
  y = yLocal + m_y0;
}

void Partition::worldToGrid(double x, double y, int &col, int &row) {
  double xLocal = x - m_x0;
  double yLocal = y - m_y0;
  localToGrid(xLocal, yLocal, col, row);
}

void Partition::gridToLocal(int col, int row, double &x, double &y) {
  if (col < 0 || col >= m_numColumns || row < 0 || row >= m_numRows)
    ROS_ERROR("gridToLocal: index out of bounds.");

  x = (col + 0.5) * m_cellSize;
  y = (row + 0.5) * m_cellSize;
}

void Partition::localToGrid(double x, double y, int &col, int &row) {
  if (x < 0 || x >= m_x1 - m_x0 || y < 0 || y >= m_y1 - m_y0)
    ROS_ERROR("localToGrid: index out of bounds.");

  col = static_cast<int>(std::floor(x / m_cellSize));
  row = static_cast<int>(std::floor(y / m_cellSize));
}

bool Partition::hasCompleteCoverage() {
  // Any non-covered free cells?
  for (auto column : m_cells) {
    for (auto cell : column) {
      if (!cell.isCovered && cell.status == Free) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace otter_coverage
