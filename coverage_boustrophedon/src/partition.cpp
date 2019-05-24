#include <cmath>
#include <coverage/partition.h>
#include <visualization_msgs/MarkerArray.h>
#include <coverage/a_star.h>

namespace otter_coverage
{

Partition::Partition() : m_initialized(false) {}

void Partition::initialize(ros::NodeHandle nh, double x0, double y0, double x1, double y1, double cellSize, double scanRange)
{
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
  std::vector<std::vector<Cell>> grid(m_width, std::vector<Cell>(m_height, Cell()));
  m_grid = grid;
}

void Partition::drawPartition(int gx, int gy)
{
  if (!m_initialized)
    return;

  int rangeCells = m_scanRange / m_cellSize + 1;

  visualization_msgs::MarkerArray markerArray;
  for (int x = std::max(0, gx - rangeCells); x < std::min(m_width, gx + rangeCells); x++)
  {
    for (int y = std::max(0, gy - rangeCells); y < std::min(m_height, gy + rangeCells); y++)
    {
      int id = m_height * x + y;

      double wx, wy;
      gridToWorld(x, y, wx, wy);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "shapes";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = wx;
      marker.pose.position.y = wy;
      marker.pose.position.z = 0.1;
      marker.scale.x = m_cellSize;
      marker.scale.y = m_cellSize;
      marker.scale.z = 0.01;

      if (m_grid[x][y].status == Free)
      {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
      }
      else if (m_grid[x][y].status == Unknown)
      {
        // Same as background color in rviz
        marker.color.r = 48.0 / 255.0;
        marker.color.g = 48.0 / 255.0;
        marker.color.b = 48.0 / 255.0;
      }
      if (m_grid[x][y].isCovered)
      {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      }
      if (m_grid[x][y].status == Blocked)
      {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
      }
      marker.color.a = 0.3f;

      marker.lifetime = ros::Duration(0.0);
      markerArray.markers.push_back(marker);
    }
  }
  m_pub.publish(markerArray);
}

// TODO: consider using costmap_2d from navigation to get smaller maps
void Partition::update(const nav_msgs::OccupancyGrid& map, double wx, double wy)
{
  // Update the status of cells based on the map received in a region around the
  // position [x,y]

  // Current cell
  int gx, gy;
  worldToGrid(wx, wy, gx, gy);

  drawPartition(gx, gy);

  // Calculate status for all nearby cells
  std::vector<Point> neighbors;
  getNeighbors(gx, gy, m_scanRange, neighbors);
  for (auto nb : neighbors)
  {
    setStatus(nb.gx, nb.gy, calcStatus(map, nb.gx, nb.gy));
  }
}

void Partition::getNeighbors(int gx, int gy, double distance,
                             std::vector<Point>& neighbors) const
{
  int maxGridDist = static_cast<int>(std::ceil(distance / (m_cellSize)));

  double wx0, wy0;
  gridToWorld(gx, gy, wx0, wy0);

  for (int gxDelta = -maxGridDist; gxDelta <= maxGridDist; gxDelta++)
  {
    for (int gyDelta = -maxGridDist; gyDelta <= maxGridDist; gyDelta++)
    {
      // Neighbors must be in the grid
      if (!withinGridBounds(gx + gxDelta, gy + gyDelta))
      {
        continue;
      }

      // Neighbors must be closer than dist from current cell
      double wx1, wy1;
      gridToWorld(gx + gxDelta, gy + gyDelta, wx1, wy1);
      if (dist(wx0, wy0, wx1, wy1) <= distance)
      {
        neighbors.push_back({gx + gxDelta, gy + gyDelta});
      }
    }
  }
}

void Partition::gridToWorld(int gx, int gy, double& wx, double& wy) const
{
  double lx;
  double ly;
  gridToLocal(gx, gy, lx, ly);
  wx = lx + m_x0;
  wy = ly + m_y0;
}

void Partition::worldToGrid(double wx, double wy, int& gx, int& gy) const
{
  double lx = wx - m_x0;
  double ly = wy - m_y0;
  localToGrid(lx, ly, gx, gy);
}

Partition::Status Partition::getStatus(int gx, int gy) const
{
  if (!withinGridBounds(gx, gy))
  {
    ROS_ERROR(
        "Partition::getStatus() - Tried to access element outside partition.");
    return Partition::Unknown;
  }
  return m_grid[gx][gy].status;
}

void Partition::setStatus(int gx, int gy, Status status)
{
  if (!withinGridBounds(gx, gy))
  {
    ROS_ERROR(
        "Partition::setStatus() - Tried to access element outside partition.");
    return;
  }
  m_grid[gx][gy].status = status;
}

bool Partition::isCovered(int gx, int gy) const
{
  if (!withinGridBounds(gx, gy))
  {
    ROS_ERROR(
        "Partition::isCovered() - Tried to access element outside partition.");
    return false;
  }
  return m_grid[gx][gy].isCovered;
}

void Partition::setCovered(int gx, int gy, bool isCovered, int coverageSize, double psi)
{
  
  if (coverageSize == 0 || std::abs(psi) < std::numeric_limits<double>::epsilon()) {
    int i = std::min(std::max(gx, 0), getWidth() - 1);
    for (int j = std::max(gy - coverageSize, 0); j < std::min(gy + coverageSize + 1, getHeight()); j++)
    {
      m_grid[i][j].isCovered = isCovered;
    }
    return;
  }
  
  psi = psi + M_PI_2; // perpendicular to moving direction
  if (isCovered) {
    Tile from = {std::min(gx + int(std::ceil(coverageSize * std::cos(psi))), getWidth() - 1), std::min(gy + int(std::ceil(coverageSize * std::sin(psi))), getHeight() - 1)};
    Tile to = {std::max(gx - int(std::ceil(coverageSize * std::cos(psi))), 0), std::max(gy - int(std::ceil(coverageSize * std::sin(psi))), 0)};
    losCover(*this, from, to);
  }
}

int Partition::getHeight() const { return m_height; }

int Partition::getWidth() const { return m_width; }

bool Partition::withinGridBounds(int gx, int gy) const
{
  return gx >= 0 && gx < m_width && gy >= 0 && gy < m_height;
}

bool Partition::withinWorldBounds(double wx, double wy) const
{
  return wx >= m_x0 && wx <= m_x1 && wy >= m_y0 && wy <= m_y1;
}

bool Partition::hasCompleteCoverage() const
{
  // Any non-covered free cells? => Not finished
  for (auto column : m_grid)
  {
    for (auto cell : column)
    {
      if (!cell.isCovered && cell.status == Free)
      {
        return false;
      }
    }
  }
  return true;
}

int mapIndex(int mx, int my, int width)
{
  // OccupancyGrid uses row major order
  return my * width + mx;
}

Partition::Status Partition::calcStatus(const nav_msgs::OccupancyGrid& map,
                                        int gx, int gy) const
{
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
  for (int i = 0; i * mapRes < m_cellSize; i++)
  {
    for (int j = 0; j * mapRes < m_cellSize; j++)
    {
      int m = mapIndex(xMap + i, yMap + j, map.info.width);

      // Map not big enough
      if (xMap + i >= map.info.width || yMap + j >= map.info.height)
      {
        unknown = true;
        continue;
      }

      // Any map cell with occupancy probability higher than a threshold
      if (map.data[m] > 50)
      {
        return Blocked;
      }

      // Any unknown map cell
      if (map.data[m] < 0)
      {
        unknown = true;
      }
    }
  }

  // No blocked cells, but some are unknown.
  if (unknown && getStatus(gx, gy) == Unknown)
    return Unknown;

  // Partially blocked cell
  if (unknown && getStatus(gx, gy) == Blocked)
    return Blocked;

  return Free;
}

void Partition::gridToLocal(int gx, int gy, double& lx, double& ly) const
{
  // if (gx < 0 || gx >= m_width || gy < 0 || gy >= m_height)
  //  ROS_ERROR("gridToLocal: index out of bounds.");

  lx = (gx + 0.5) * m_cellSize;
  ly = (gy + 0.5) * m_cellSize;
}

void Partition::localToGrid(double lx, double ly, int& gx, int& gy) const
{
  // if (lx < 0 || lx >= m_x1 - m_x0 || ly < 0 || ly >= m_y1 - m_y0)
  //  ROS_ERROR("localToGrid: index out of bounds.");

  gx = static_cast<int>(std::floor(lx / m_cellSize));
  gy = static_cast<int>(std::floor(ly / m_cellSize));
}

} // namespace otter_coverage
