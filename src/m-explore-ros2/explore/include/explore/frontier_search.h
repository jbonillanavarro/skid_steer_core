#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  double min_distance;
  double cost;
  // heading (map frame) the robot actually has to take to start moving
  // towards this frontier, derived from the real BFS path, not from the
  // straight line to the centroid. NaN if it could not be established
  // (frontier basically on top of the robot).
  double approach_yaw;
  geometry_msgs::msg::Point initial;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point middle;
  std::vector<geometry_msgs::msg::Point> points;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch() : logger_(rclcpp::get_logger("frontier_search")) {} // Default constructor for the logger

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(nav2_costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale, double min_frontier_size,
                 rclcpp::Logger logger);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::msg::Point position);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag,
                            double path_distance, double approach_yaw);

  /**
   * @brief Heading the robot must actually take to head for a frontier
   * @details Reconstructs the real route from the BFS parent chain and
   * returns the bearing to the furthest point of it still in direct line of
   * sight, i.e. the first leg of a string-pulled path. In open space that
   * point is the frontier itself, so this degrades gracefully to the
   * straight-line bearing; behind a corner it is the corner, which is where
   * the robot really has to point first.
   * @param entry_cell Last traversable cell of the path, adjacent to the
   * frontier
   * @param parent BFS parent of every visited cell
   * @param origin Robot position in world coordinates
   * @return heading in radians (map frame), NaN if degenerate
   */
  double approachYaw(unsigned int entry_cell,
                     const std::vector<unsigned int>& parent,
                     const geometry_msgs::msg::Point& origin);

  /**
   * @brief Checks whether two cells are joined by an unobstructed straight
   * line of traversable cells (Bresenham). Unknown cells block, since we
   * cannot claim to see through them.
   */
  bool lineOfSight(unsigned int from_cell, unsigned int to_cell);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

private:
  nav2_costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
  rclcpp::Logger logger_;
};
}  // namespace frontier_exploration
#endif
