#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::MAX_NON_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size, rclcpp::Logger logger)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , logger_(logger)
{
}

std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(logger_, "[FrontierSearch] Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);
  // real path distance (through free cells only, following the 4-connected
  // BFS) from the robot to each visited cell -- used instead of straight
  // line distance so a frontier hidden behind a wall doesn't look "close"
  // just because it is close as the crow flies.
  std::vector<double> path_distance(size_x_ * size_y_, 0.0);
  // BFS predecessor of every visited cell, so the actual path back to the
  // robot can be reconstructed (used to work out which way the robot really
  // has to turn to head for each frontier). The root points to itself.
  std::vector<unsigned int> parent(size_x_ * size_y_, 0);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(logger_, "[FrontierSearch] Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;
  path_distance[bfs.front()] = 0.0;
  parent[bfs.front()] = bfs.front();

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all non-obstacle, unvisited cells (any inflated cost
      // below the inscribed-obstacle threshold is still traversable, same
      // convention nav2 planners use, not just strictly non-increasing cost)
      if (map_[nbr] <= MAX_NON_OBSTACLE && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        path_distance[nbr] = path_distance[idx] + costmap_->getResolution();
        parent[nbr] = idx;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        double entry_distance = path_distance[idx] + costmap_->getResolution();
        // idx is the last traversable cell of the path to this frontier, so
        // the parent chain from it is the real route the robot would take.
        double approach_yaw = approachYaw(idx, parent, position);
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag,
                                                 entry_distance, approach_yaw);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

double FrontierSearch::approachYaw(unsigned int entry_cell,
                                   const std::vector<unsigned int>& parent,
                                   const geometry_msgs::msg::Point& origin)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  unsigned int robot_mx, robot_my;
  if (!costmap_->worldToMap(origin.x, origin.y, robot_mx, robot_my)) {
    return nan;
  }
  const unsigned int robot_cell = costmap_->getIndex(robot_mx, robot_my);

  // rebuild the route the robot would actually walk, robot end first
  std::vector<unsigned int> route;
  for (unsigned int cell = entry_cell;; cell = parent[cell]) {
    route.push_back(cell);
    if (parent[cell] == cell) {
      break;
    }
  }
  std::reverse(route.begin(), route.end());

  // Walk that route outwards and keep the furthest point still reachable in
  // a straight line: the first leg of a string-pulled path. Around a corner
  // it lands on the corner, which is where the robot really has to point
  // first; in the open it walks all the way to the frontier, so this reduces
  // to the straight-line bearing and nothing is lost where the old formula
  // was already right.
  //
  // Note this must not be cut short at some fixed lookahead distance. The
  // 4-connected BFS reaches a diagonal target by going all the way along one
  // axis and then the other, so any point picked in the middle of the route
  // sits on the corner of that L and reports a heading that is an artifact
  // of the search, not of the map (measured: 3 deg instead of 45 on an open
  // diagonal). Only the *furthest visible* point is free of it, because a
  // staircase corner is never the furthest visible point when the far end of
  // the leg is visible too.
  const size_t step =
      std::max<size_t>(1, static_cast<size_t>(0.25 / costmap_->getResolution()));
  unsigned int candidate = route.front();
  size_t i = step;
  for (; i < route.size(); i += step) {
    if (!lineOfSight(robot_cell, route[i])) {
      break;
    }
    candidate = route[i];
  }
  // whole route visible: aim at the frontier itself rather than at whatever
  // the last sampled step happened to land on
  if (i >= route.size() && lineOfSight(robot_cell, route.back())) {
    candidate = route.back();
  }

  unsigned int cx, cy;
  double wx, wy;
  costmap_->indexToCells(candidate, cx, cy);
  costmap_->mapToWorld(cx, cy, wx, wy);
  double dx = wx - origin.x;
  double dy = wy - origin.y;
  // too close to the robot to read a meaningful bearing off it
  if (dx * dx + dy * dy < costmap_->getResolution() * costmap_->getResolution()) {
    return nan;
  }
  return std::atan2(dy, dx);
}

bool FrontierSearch::lineOfSight(unsigned int from_cell, unsigned int to_cell)
{
  unsigned int ux0, uy0, ux1, uy1;
  costmap_->indexToCells(from_cell, ux0, uy0);
  costmap_->indexToCells(to_cell, ux1, uy1);

  int x = static_cast<int>(ux0), y = static_cast<int>(uy0);
  const int x1 = static_cast<int>(ux1), y1 = static_cast<int>(uy1);
  const int dx = std::abs(x1 - x);
  const int dy = -std::abs(y1 - y);
  const int sx = (x < x1) ? 1 : -1;
  const int sy = (y < y1) ? 1 : -1;
  int err = dx + dy;

  // the origin cell is skipped on purpose: the robot may well be standing on
  // an inflated cell, and that must not make everything invisible.
  while (x != x1 || y != y1) {
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
    if (map_[costmap_->getIndex(x, y)] > MAX_NON_OBSTACLE) {
      return false;
    }
  }
  return true;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag,
                                          double path_distance,
                                          double approach_yaw)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  // real distance through free cells to this frontier's entry point (the
  // closest point of the frontier to the robot, by construction of the
  // outer BFS), instead of straight-line distance which ignores walls.
  output.min_distance = path_distance;
  output.approach_yaw = approach_yaw;
  double closest_local = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // pick a representative "middle" point of this frontier cluster
        // (purely local choice within the cluster, straight-line distance
        // is fine here since it never crosses a wall -- the cluster is one
        // connected blob of unknown cells). output.min_distance itself is
        // NOT touched here anymore: it already holds the real path
        // distance to the frontier's entry point, set from the outer BFS.
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < closest_local) {
          closest_local = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}  // namespace frontier_exploration
