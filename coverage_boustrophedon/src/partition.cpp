#include <coverage/partition.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>

namespace otter_coverage {

double dist(double x0, double y0, double x1, double y1) {
  return std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

Partition::Partition() : m_initialized(false) {}

void Partition::initialize(ros::NodeHandle nh, double x0, double y0, double x1,
                           double y1, double cellSize, double scanRange) {
  // TODO: rotate coordinate system? so that rectangle doesn't need to be
  // aligned with x and y axis in map frame.

  m_initialized = true;

  m_nh = nh;
  m_pub = m_nh.advertise<visualization_msgs::MarkerArray>("cell_partition", 1);

  m_x0 = x0;
  m_y0 = y0;
  m_x1 = x1;
  m_y1 = y1;
  m_cellSize = cellSize;
  m_scanRange = scanRange;

  m_width = static_cast<int>(std::ceil((x1 - x0) / cellSize));
  m_height = static_cast<int>(std::ceil((y1 - y0) / cellSize));

  // Initialize cells
  std::vector<std::vector<Cell>> grid(m_width,
                                      std::vector<Cell>(m_height, Cell()));
  m_grid = grid;
}

void Partition::drawPartition() {
  if (!m_initialized) return;

  visualization_msgs::MarkerArray markerArray;
  int id = 0;
  for (int gx = 0; gx < m_width; gx++) {
    for (int gy = 0; gy < m_height; gy++) {
      double wx, wy;
      gridToWorld(gx, gy, wx, wy);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "shapes";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = wx;
      marker.pose.position.y = wy;
      marker.scale.x = m_cellSize;
      marker.scale.y = m_cellSize;
      marker.scale.z = 0.01;

      if (m_grid[gx][gy].status == Blocked) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
      } else if (m_grid[gx][gy].status == Free) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
      }
      if (m_grid[gx][gy].status == Unknown) {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
      }
      if (m_grid[gx][gy].isCovered) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      }
      marker.color.a = 0.1f;

      marker.lifetime = ros::Duration(0.0);
      markerArray.markers.push_back(marker);
    }
  }
  m_pub.publish(markerArray);
}

// TODO: consider using costmap_2d from navigation to get smaller maps
void Partition::update(const nav_msgs::OccupancyGrid &map, double wx,
                       double wy) {
  // Update the status of cells based on the map received in a region around the
  // position [x,y]

  drawPartition();

  // Current cell
  int gx, gy;
  worldToGrid(wx, wy, gx, gy);

  // Calculate status for all nearby cells
  std::vector<Point> neighbors;
  getNeighbors(gx, gy, m_scanRange, neighbors);
  for (auto nb : neighbors) {
    setStatus(nb.gx, nb.gy, calcStatus(map, nb.gx, nb.gy));
  }
}

void Partition::getNeighbors(int gx, int gy, double distance,
                             std::vector<Point> &neighbors) {
  int maxGridDist = static_cast<int>(std::ceil(distance / (m_cellSize)));

  double wx0, wy0;
  gridToWorld(gx, gy, wx0, wy0);

  for (int gxDelta = -maxGridDist; gxDelta <= maxGridDist; gxDelta++) {
    for (int gyDelta = -maxGridDist; gyDelta <= maxGridDist; gyDelta++) {
      // Neighbors must be in the grid
      if (!withinGridBounds(gx + gxDelta, gy + gyDelta)) {
        continue;
      }

      // Neighbors must be closer than dist from current cell
      double wx1, wy1;
      gridToWorld(gx + gxDelta, gy + gyDelta, wx1, wy1);
      if (dist(wx0, wy0, wx1, wy1) <= distance) {
        neighbors.push_back({gx + gxDelta, gy + gyDelta});
      }
    }
  }
}

void Partition::gridToWorld(int gx, int gy, double &wx, double &wy) {
  double lx;
  double ly;
  gridToLocal(gx, gy, lx, ly);
  wx = lx + m_x0;
  wy = ly + m_y0;
}

void Partition::worldToGrid(double wx, double wy, int &gx, int &gy) {
  double lx = wx - m_x0;
  double ly = wy - m_y0;
  localToGrid(lx, ly, gx, gy);
}

Partition::Status Partition::getStatus(int gx, int gy) {
  if (!withinGridBounds(gx, gy)) {
    ROS_ERROR(
        "Partition::getStatus() - Tried to access element outside partition.");
    return Partition::Unknown;
  }
  return m_grid[gx][gy].status;
}

void Partition::setStatus(int gx, int gy, Status status) {
  if (!withinGridBounds(gx, gy)) {
    ROS_ERROR(
        "Partition::setStatus() - Tried to access element outside partition.");
    return;
  }
  m_grid[gx][gy].status = status;
}

bool Partition::isCovered(int gx, int gy) {
  if (!withinGridBounds(gx, gy)) {
    ROS_ERROR(
        "Partition::isCovered() - Tried to access element outside partition.");
    return false;
  }
  return m_grid[gx][gy].isCovered;
}

void Partition::setCovered(int gx, int gy, bool isCovered) {
  if (!withinGridBounds(gx, gy)) {
    ROS_ERROR(
        "Partition::setCovered() - Tried to access element outside partition.");
    return;
  }
  m_grid[gx][gy].isCovered = isCovered;
}

int Partition::getHeight() const { return m_height; }

int Partition::getWidth() const { return m_width; }

bool Partition::withinGridBounds(int gx, int gy) {
  return gx >= 0 && gx < m_width && gy >= 0 && gy < m_height;
}

bool Partition::withinWorldBounds(double wx, double wy) {
  return wx >= m_x0 && wx <= m_x1 && wy >= m_y0 && wy <= m_y1;
}

bool Partition::hasCompleteCoverage() {
  // Any non-covered free cells?
  for (auto column : m_grid) {
    for (auto cell : column) {
      if (!cell.isCovered && cell.status == Free) {
        return false;
      }
    }
  }
  return true;
}

int mapIndex(int mx, int my, int width) {
  // OccupancyGrid uses row major order
  return my * width + mx;
}

Partition::Status Partition::calcStatus(const nav_msgs::OccupancyGrid &map,
                                        int gx, int gy) {
  /* A cell in the partition contains many cells in the map. All
   * map cells in a partition cell are free => partition cell is free
   */

  double xOrigin = map.info.origin.position.x;
  double yOrigin = map.info.origin.position.y;
  double mapRes = static_cast<double>(map.info.resolution);

  // Find position of partition cell in map frame
  double xWorld, yWorld;
  gridToWorld(gx, gy, xWorld, yWorld);

  // Find x and y in map of lower right corner of surrounding square
  int xMap = static_cast<int>(((xWorld - m_cellSize / 2) - xOrigin) / mapRes);
  int yMap = static_cast<int>(((yWorld - m_cellSize / 2) - yOrigin) / mapRes);

  // Check if all map cells in the partition cell is free
  bool unknown = false;
  for (int i = 0; i * mapRes < m_cellSize; i++) {
    for (int j = 0; j * mapRes < m_cellSize; j++) {
      int m = mapIndex(xMap + i, yMap + j, map.info.width);

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

void Partition::gridToLocal(int gx, int gy, double &lx, double &ly) {
  if (gx < 0 || gx >= m_width || gy < 0 || gy >= m_height)
    ROS_ERROR("gridToLocal: index out of bounds.");

  lx = (gx + 0.5) * m_cellSize;
  ly = (gy + 0.5) * m_cellSize;
}

void Partition::localToGrid(double lx, double ly, int &gx, int &gy) {
  if (lx < 0 || lx >= m_x1 - m_x0 || ly < 0 || ly >= m_y1 - m_y0)
    ROS_ERROR("localToGrid: index out of bounds.");

  gx = static_cast<int>(std::floor(lx / m_cellSize));
  gy = static_cast<int>(std::floor(ly / m_cellSize));
}

}  // namespace otter_coverage
