// Tests for the BFS-derived approach heading of a frontier.
//
// The property under test: Frontier::approach_yaw must be the direction the
// robot actually has to set off in, following the real route through free
// cells, and not the straight line to the frontier -- while still collapsing
// to the straight line when there is nothing in the way.

#include <gtest/gtest.h>

#include <cmath>

#include <explore/frontier_search.h>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

using frontier_exploration::Frontier;
using frontier_exploration::FrontierSearch;

namespace
{
constexpr double kRes = 0.1;
constexpr unsigned int kCells = 60;

// exposes the same cost parameters the node uses, minus the shaping done in
// explore.cpp, which is irrelevant here
FrontierSearch makeSearch(nav2_costmap_2d::Costmap2D& map)
{
  return FrontierSearch(&map, /*potential_scale=*/1.0, /*gain_scale=*/1.0,
                        /*min_frontier_size=*/0.0,
                        rclcpp::get_logger("test_frontier_search"));
}

void fill(nav2_costmap_2d::Costmap2D& map, unsigned int x0, unsigned int y0,
          unsigned int x1, unsigned int y1, unsigned char value)
{
  for (unsigned int x = x0; x <= x1; ++x) {
    for (unsigned int y = y0; y <= y1; ++y) {
      map.setCost(x, y, value);
    }
  }
}

geometry_msgs::msg::Point worldPoint(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

double degrees(double radians)
{
  return radians * 180.0 / M_PI;
}

// signed difference between two headings, in degrees
double headingErrorDeg(double a, double b)
{
  double d = std::atan2(std::sin(a - b), std::cos(a - b));
  return std::abs(degrees(d));
}
}  // namespace

// A U-shaped corridor: the frontier sits almost due north of the robot, but
// the only route to it runs east, then north, then back west. The straight
// line is therefore a lie, and it is exactly the lie that made the robot
// think a frontier behind a wall was "ahead".
TEST(FrontierSearchApproachYaw, RouteAroundWallBeatsStraightLine)
{
  nav2_costmap_2d::Costmap2D map(kCells, kCells, kRes, 0.0, 0.0,
                                 nav2_costmap_2d::LETHAL_OBSTACLE);

  // bottom corridor (robot starts at its west end), east wing, top corridor
  fill(map, 5, 5, 45, 7, nav2_costmap_2d::FREE_SPACE);
  fill(map, 43, 5, 45, 45, nav2_costmap_2d::FREE_SPACE);
  fill(map, 12, 43, 45, 45, nav2_costmap_2d::FREE_SPACE);
  // unexplored area at the far (west) end of the top corridor
  fill(map, 8, 43, 11, 45, nav2_costmap_2d::NO_INFORMATION);

  auto search = makeSearch(map);
  auto robot = worldPoint(0.6, 0.6);  // cell (6, 6)
  auto frontiers = search.searchFrom(robot);

  ASSERT_EQ(frontiers.size(), 1u);
  const Frontier& f = frontiers.front();
  ASSERT_TRUE(std::isfinite(f.approach_yaw));

  double straight_line_yaw =
      std::atan2(f.centroid.y - robot.y, f.centroid.x - robot.x);

  // the straight line points north-ish, straight through two walls
  EXPECT_GT(headingErrorDeg(straight_line_yaw, 0.0), 60.0)
      << "test setup is wrong: straight line should not already point east";
  // the real route sets off due east
  EXPECT_LT(headingErrorDeg(f.approach_yaw, 0.0), 20.0)
      << "approach_yaw=" << degrees(f.approach_yaw) << " deg, expected ~0";

  // and the distance is the route, not the crow flight
  EXPECT_GT(f.min_distance, 6.0);
}

// A dead end: the robot faces east into a pocket, and the only frontier is
// back the way it came. The heading has to come out as a U-turn.
TEST(FrontierSearchApproachYaw, DeadEndReportsUTurn)
{
  nav2_costmap_2d::Costmap2D map(kCells, kCells, kRes, 0.0, 0.0,
                                 nav2_costmap_2d::LETHAL_OBSTACLE);

  fill(map, 5, 28, 40, 31, nav2_costmap_2d::FREE_SPACE);
  // unexplored area only at the west end; everything east of the robot is a
  // known, closed pocket
  fill(map, 2, 28, 4, 31, nav2_costmap_2d::NO_INFORMATION);

  auto search = makeSearch(map);
  auto robot = worldPoint(3.5, 2.95);  // cell (35, 29), deep in the pocket
  auto frontiers = search.searchFrom(robot);

  ASSERT_EQ(frontiers.size(), 1u);
  const Frontier& f = frontiers.front();
  ASSERT_TRUE(std::isfinite(f.approach_yaw));

  // robot heading east (yaw 0) has to turn all the way around
  EXPECT_GT(headingErrorDeg(f.approach_yaw, 0.0), 150.0)
      << "approach_yaw=" << degrees(f.approach_yaw) << " deg, expected ~180";
}

// Nothing in the way: the heading must fall back to the straight line rather
// than to the 4-connected staircase the BFS walks (which would quantise it
// to multiples of 90 degrees and be worse than what we had before).
TEST(FrontierSearchApproachYaw, OpenSpaceMatchesStraightLine)
{
  nav2_costmap_2d::Costmap2D map(kCells, kCells, kRes, 0.0, 0.0,
                                 nav2_costmap_2d::FREE_SPACE);

  // unexplored quadrant up and to the right, so the frontier lies on a
  // diagonal -- the worst case for a 4-connected path
  fill(map, 40, 40, kCells - 1, kCells - 1, nav2_costmap_2d::NO_INFORMATION);

  auto search = makeSearch(map);
  auto robot = worldPoint(1.0, 1.0);  // cell (10, 10)
  auto frontiers = search.searchFrom(robot);

  ASSERT_FALSE(frontiers.empty());
  const Frontier& f = frontiers.front();
  ASSERT_TRUE(std::isfinite(f.approach_yaw));

  // the nearest corner of the unknown quadrant is at 45 degrees
  EXPECT_LT(headingErrorDeg(f.approach_yaw, M_PI / 4.0), 15.0)
      << "approach_yaw=" << degrees(f.approach_yaw)
      << " deg, expected ~45 (staircase artifact if it comes out near 0 or 90)";
}
