#include <coverage_binn/partition_binn.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>

PartitionBinn::PartitionBinn() {}

PartitionBinn::PartitionBinn(ros::NodeHandle nh) : m_initialized(false) {
  ROS_INFO("PartitionBinn constructed.");

  m_nh = nh;
  m_pub = m_nh.advertise<visualization_msgs::MarkerArray>("cell_partition", 1);
}

void PartitionBinn::initialize(double x0, double y0, double x1, double y1,
                               double rc) {
  // TODO: rotate coordinate system? so that rectangle doesn't need to be
  // aligned with x and y axis in map frame.

  ROS_INFO("PartitionBinn initialized.");

  m_initialized = true;

  m_x0 = x0;
  m_y0 = y0;
  m_x1 = x1;
  m_y1 = y1;
  m_rc = rc;

  // Find the size of the workspace
  m_Xw = x1 - x0;
  m_Yw = y1 - y0;

  // Find number of columns m
  if (std::fmod(m_Xw / (3.0 / 2.0 * rc), 1.0) <= 2.0 / 3.0) {
    m_m = static_cast<int>(std::floor(m_Xw / (3.0 / 2.0 * rc))) + 1;
  } else if (std::fmod(m_Xw / (3.0 / 2.0 * rc), 1.0) > 2.0 / 3.0) {
    m_m = static_cast<int>(std::floor(m_Xw / (3.0 / 2.0 * rc))) + 2;
  }

  // Find number of rows nl in each column
  std::vector<int> n;
  for (int l = 1; l <= m_m; l++) {
    if (std::fmod(m_Yw / (std::sqrt(3.0) * rc), 1.0) <= 1.0 / 2.0) {
      n.push_back(static_cast<int>(std::floor(m_Yw / (std::sqrt(3.0) * rc))) +
                  1);
    } else if (std::fmod(m_Yw / (std::sqrt(3.0) * rc), 1.0) > 1.0 / 2.0) {
      n.push_back(static_cast<int>(std::floor(m_Yw / (std::sqrt(3.0) * rc))) +
                  1 + (l % 2));
    }
  }
  m_n = n;

  // Initialize cells
  std::vector<std::vector<Cell>> cells;
  for (int l = 1; l <= m_m; l++) {
    std::vector<Cell> column;
    for (int k = 1; k <= m_n[l - 1]; k++) {
      Cell cell;
      column.push_back(cell);
    }
    cells.push_back(column);
  }
  m_cells = cells;
}

void PartitionBinn::drawPartition() {
  visualization_msgs::MarkerArray ma;
  int id = 0;
  for (int l = 1; l <= m_m; l++) {
    for (int k = 1; k <= m_n[l - 1]; k++) {
      double x;
      double y;
      gridToWorld(l, k, x, y);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "shapes";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.scale.x = m_rc * 2;
      marker.scale.y = m_rc * 2;
      marker.scale.z = 0.01;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.3f;
      marker.lifetime = ros::Duration(2.0);
      ma.markers.push_back(marker);
    }
  }
  m_pub.publish(ma);
}

void PartitionBinn::update(const nav_msgs::OccupancyGrid &map, double x,
                           double y) {
  // Update the status of cells based on the map received in a region around the
  // position [x,y]

  drawPartition();

  // Current cell
  int l;
  int k;
  worldToGrid(x, y, l, k);

  // Calculate status for all nearby cells
  // CellStatus status = calculateStatus(map, l + 1, k);
  // m_cells[l + 1][k].status = status;
}

unsigned int mapIndex(unsigned int x, unsigned int y, unsigned int width) {
  return y * width + x;
}

PartitionBinn::CellStatus PartitionBinn::calculateStatus(
    const nav_msgs::OccupancyGrid &map, int l, int k) {
  /* A cell in the partition contains many cells in the map. All
   * map cells in a partition cell are free => partition cell is free
   */

  // Find position of partition cell in map frame
  double xWorld;
  double yWorld;
  gridToWorld(l, k, xWorld, yWorld);

  // Find x and y in map of center of cell
  unsigned int xMap_c = static_cast<unsigned int>(
      (xWorld - map.info.origin.position.x) / map.info.resolution);
  unsigned int yMap_c = static_cast<unsigned int>(
      (yWorld - map.info.origin.position.y) / map.info.resolution);

  // Find x and y in map of lower left corner of surrounding square
  unsigned int xMap = static_cast<unsigned int>(
      ((xWorld - m_rc) - map.info.origin.position.x) / map.info.resolution);
  unsigned int yMap = static_cast<unsigned int>(
      ((yWorld - m_rc) - map.info.origin.position.y) / map.info.resolution);

  // Check if all map cells in the partition cell is free
  for (unsigned int i = 0; i * map.info.resolution < 2 * m_rc; i++) {
    for (unsigned int j = 0; j * map.info.resolution < 2 * m_rc; j++) {
      unsigned int m = mapIndex(xMap + i, yMap + j, map.info.width);

      // Map cell not in partition cell (circle shape)
      if (std::pow((xMap + i - xMap_c) * map.info.resolution, 2) +
              std::pow((yMap + j - yMap_c) * map.info.resolution, 2) >
          m_rc * m_rc) {
        continue;
      }

      // Map not big enough
      if (xMap + i >= map.info.height || yMap + j >= map.info.width) {
        ROS_INFO("Cell unknown");
        return Unknown;
      }

      // Any map cell with occupancy probability > 50
      if (map.data[m] > 50) {
        return Blocked;
      }

      // Any unknown map cell
      // if (map.data[m] < 0) {
      //  return Unknown;
      //}
    }
  }

  return Free;
}

void PartitionBinn::gridToWorld(int l, int k, double &xc, double &yc) {
  double xLocal;
  double yLocal;
  gridToLocal(l, k, xLocal, yLocal);
  xc = xLocal + m_x0;
  yc = yLocal + m_y0;
}

void PartitionBinn::worldToGrid(double xc, double yc, int &l, int &k) {
  double xLocal = xc - m_x0;
  double yLocal = yc - m_y0;
  localToGrid(xLocal, yLocal, l, k);
}

void PartitionBinn::gridToLocal(int l, int k, double &xc, double &yc) {
  if (l < 1 || l > m_m || k < 0 || k > m_n[l - 1])
    ROS_ERROR("gridToLocal: index out of bounds.");

  if (l % 2 == 1) {
    xc = (3.0 / 2.0 * l - 1.0) * m_rc;
    yc = (k - 1.0) * std::sqrt(3) * m_rc;
  } else {  // l % 2 == 0
    xc = (3.0 / 2.0 * l - 1.0) * m_rc;
    yc = (k - 1.0 / 2.0) * std::sqrt(3) * m_rc;
  }
}

// TODO: Not accurate, circles are treated as squares. Maybe do better.
void PartitionBinn::localToGrid(double xc, double yc, int &l, int &k) {
  if (xc < 0 || xc > m_Xw || yc < 0 || yc > m_Yw)
    ROS_ERROR("localToGrid: index out of bounds.");

  // Find column

  l = static_cast<int>(
          std::lround((std::max(xc - 0.5 * m_rc, 0.0)) / (3.0 / 2.0 * m_rc))) +
      1;

  // Find row
  if (l % 2 == 0) {
    k = static_cast<int>(
            std::lround((std::max(yc, 0.0)) / (std::sqrt(3.0) * m_rc))) +
        1;
  } else {  // l % 2 == 1
    k = static_cast<int>(
            std::lround((std::max(yc, 0.0)) / (std::sqrt(3.0) * m_rc))) +
        1;
  }
}
