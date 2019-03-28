#ifndef OTTER_A_STAR_H_
#define OTTER_A_STAR_H_

#include <coverage/partition.h>
#include <deque>
#include <vector>

namespace otter_coverage
{

/* Declarations */

struct Tile
{
  int gx;
  int gy;
};

struct Node
{
  Node() : gx(-1), gy(-1), pgx(-1), pgy(-1) {}
  Node(int _gx, int _gy, int _pgx, int _pgy)
      : gx(_gx), gy(_gy), pgx(_pgx), pgy(_pgy)
  {
  }
  bool exists() { return gx != -1; }
  int gx, gy;
  int pgx, pgy;
  double f, g, h;
};

/* Implementations */

static void aStarNeighbor(Partition partition, Node q, int dx, int dy,
                          std::map<std::pair<int, int>, Node> closed,
                          std::map<std::pair<int, int>, Node>& open)
{
  int ngx = q.gx + dx;
  int ngy = q.gy + dy;
  if (partition.withinGridBounds(ngx, ngy) && partition.isCovered(ngx, ngy) && partition.getStatus(ngx, ngy) == Partition::Free)
  {
    Node node(ngx, ngy, q.gx, q.gy);
    node.g = q.g + 1;
    node.h = partition.dist(node.gx, node.gy, node.pgx, node.pgy);
    node.f = node.g + node.h;
    try
    {
      if (open.at({ngx, ngy}).f < node.f)
      {
        return;
      }
    }
    catch (std::out_of_range)
    {
    }
    try
    {
      if (closed.at({ngx, ngy}).f < node.f)
      {
        return;
      }
    }
    catch (std::out_of_range)
    {
    }

    open[{node.gx, node.gy}] = node;
  }
}

static std::vector<Tile> aStarSearch(Partition partition, Tile from, Tile to)
{

  std::map<std::pair<int, int>, Node> open;
  std::map<std::pair<int, int>, Node> closed;

  open[{from.gx, from.gy}] = Node(from.gx, from.gy, from.gx, from.gy);

  while (!open.empty())
  {
    // Pop tile with lowest f-value from open
    std::pair<int, int> minPos = {-1, -1};
    for (auto it = open.begin(); it != open.end(); it++)
    {
      if (minPos.first < 0 || it->second.f < open[minPos].f)
      {
        minPos = it->first;
      }
    }
    Node q = open[minPos];
    open.erase(minPos);

    // Check if finished
    if (q.gx == to.gx && q.gy == to.gy)
    {
      // Create path
      std::vector<Tile> reversePath;
      reversePath.push_back({q.gx, q.gy});
      // Add parent of each node to path.
      Node node = q;
      while (node.gx != from.gx || node.gy != from.gy)
      {
        reversePath.push_back({node.pgx, node.pgy});
        node = closed[{node.pgx, node.pgy}];
      }

      std::vector<Tile> path;
      for (auto it = reversePath.rbegin(); it != reversePath.rend(); it++)
      {
        path.push_back(*it);
      }
      return path;
    }

    // Neighbors
    aStarNeighbor(partition, q, 1, 0, closed, open);
    aStarNeighbor(partition, q, -1, 0, closed, open);
    aStarNeighbor(partition, q, 0, 1, closed, open);
    aStarNeighbor(partition, q, 0, -1, closed, open);
    aStarNeighbor(partition, q, 1, 1, closed, open);
    aStarNeighbor(partition, q, -1, 1, closed, open);
    aStarNeighbor(partition, q, -1, 1, closed, open);
    aStarNeighbor(partition, q, -1, -1, closed, open);

    // Push q to closed
    closed[{q.gx, q.gy}] = q;
  }

  return std::vector<Tile>(); // No path found
}

static bool lineOfSight(Partition partition, Tile from, Tile to)
{ // Algorithm: http://eugen.dedu.free.fr/projects/bresenham/
  int x1 = from.gx;
  int y1 = from.gy;
  int x2 = to.gx;
  int y2 = to.gy;

  int i;              // loop counter
  int ystep, xstep;   // the step on y and x axis
  int error;          // the error accumulated during the increment
  int errorprev;      // *vision the previous value of the error variable
  int y = y1, x = x1; // the line points
  int ddy, ddx;       // compulsory variables: the double values of dy and dx
  int dx = x2 - x1;
  int dy = y2 - y1;
  if (partition.getStatus(x1, y1) != Partition::Free) // first point
  {
    return false;
  }
  // NB the last point can't be here, because of its previous point (which has
  // to be verified)
  if (dy < 0)
  {
    ystep = -1;
    dy = -dy;
  }
  else
    ystep = 1;
  if (dx < 0)
  {
    xstep = -1;
    dx = -dx;
  }
  else
    xstep = 1;
  ddy = 2 * dy; // work with double values for full precision
  ddx = 2 * dx;
  if (ddx >= ddy) // first octant (0 <= slope <= 1)

  {
    // compulsory initialization (even for errorprev, needed when dx==dy)
    errorprev = error = dx; // start in the middle of the square
    for (i = 0; i < dx; i++)
    { // do not use the first point (already done)
      x += xstep;
      error += ddy;
      if (error > ddx)
      { // increment y if AFTER the middle ( > )
        y += ystep;
        error -= ddx;
        // three cases (octant == right->right-top for directions below):
        if (error + errorprev < ddx) // bottom square also
        {
          if (partition.getStatus(x, y - ystep) != Partition::Free)
          {
            return false;
          }
        }
        else if (error + errorprev > ddx) // left square also
        {
          if (partition.getStatus(x - xstep, y) != Partition::Free)
          {
            return false;
          }
        }
        else // corner: bottom and left squares also
        {
          if (partition.getStatus(x, y - ystep) != Partition::Free)
          {
            return false;
          }
          if (partition.getStatus(x - xstep, y) != Partition::Free)
          {
            return false;
          }
        }
      }
      if (partition.getStatus(x, y) != Partition::Free)
      {
        return false;
      }
      errorprev = error;
    }
  }
  else
  { // the same as above
    errorprev = error = dy;
    for (i = 0; i < dy; i++)
    {
      y += ystep;
      error += ddx;
      if (error > ddy)
      {
        x += xstep;
        error -= ddy;
        if (error + errorprev < ddy)
        {
          if (partition.getStatus(x - xstep, y) != Partition::Free)
          {
            return false;
          }
        }
        else if (error + errorprev > ddy)
        {
          if (partition.getStatus(x, y - ystep) != Partition::Free)
          {
            return false;
          }
        }
        else
        {
          if (partition.getStatus(x - xstep, y) != Partition::Free)
          {
            return false;
          }
          if (partition.getStatus(x, y - ystep) != Partition::Free)
          {
            return false;
          }
        }
      }
      if (partition.getStatus(x, y) != Partition::Free)
      {
        return false;
      }
      errorprev = error;
    }
  }
  // assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the
  // same with the last point of the algorithm
  return true;
}

static std::vector<Tile> aStarSPT(Partition partition, Tile from, Tile to)
{
  auto aStarPath = aStarSearch(partition, from, to);
  if (aStarPath.size() <= 2)
  {
    return aStarPath;
  }
  std::vector<Tile> smoothedPath = {aStarPath.front()};

  unsigned int k = 0;
  for (unsigned int i = 0; i < aStarPath.size(); i++)
  {
    for (auto it = aStarPath.rbegin(); it != aStarPath.rend(); it++)
    {
      if (lineOfSight(partition, smoothedPath[k], *it))
      {
        smoothedPath.push_back(*it);
        k++;
        if (it == aStarPath.rbegin())
        {
          return smoothedPath;
        }
        else
        {
          break;
        }
      }
    }
  }

  // Should not happen
  return aStarPath;
}

} // namespace otter_coverage

#endif
