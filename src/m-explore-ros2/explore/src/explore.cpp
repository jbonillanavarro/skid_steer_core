/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

// Identity test for "this is still the same goal/frontier as before".
// The tolerance matters more than it looks: a frontier's centroid drifts by
// tens of centimetres between planning cycles as the sensor eats into the
// unknown region and the cluster grows, splits or merges. Comparing it at
// centimetre precision means the goal is treated as brand new almost every
// cycle, which silently disables any commitment logic and re-sends goals to
// Nav2 for no reason. Tolerance is a parameter (goal_identity_tolerance).
inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two,
                              double tolerance)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < tolerance;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("commitment_scale", 0.0);
  this->declare_parameter<float>("commitment_hysteresis", 0.0);
  this->declare_parameter<float>("commitment_max", 1e6);
  this->declare_parameter<float>("goal_identity_tolerance", 0.01);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("commitment_scale", commitment_scale_);
  this->get_parameter("commitment_hysteresis", commitment_hysteresis_);
  this->get_parameter("commitment_max", commitment_max_);
  this->get_parameter("goal_identity_tolerance", goal_identity_tolerance_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);

  // penalize frontiers that require a sharp turn from the robot's current
  // heading, so it prefers continuing forward over spinning to chase a
  // marginally cheaper frontier off to the side or behind.
  //
  // also reward the frontier we are already pursuing, both with a fixed head
  // start the moment it is picked (commitment_hysteresis_) and with a bonus
  // that grows as real progress is made towards it. the fixed part is what
  // resolves ambiguity at the moment of choosing: when every option is far
  // away the costs sit close together and jitter between cycles, so without
  // a margin the ranking flips on noise and the robot dithers instead of
  // committing.
  const double yaw = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);

  // First pass: how much the robot really has to turn for each frontier, and
  // what the cheapest turn on offer is. The penalty below is charged on the
  // DIFFERENCE against that cheapest turn, not on the absolute angle.
  //
  // This is the difference between "turning is expensive" and "turning when
  // you didn't have to is expensive", and only the second one is what we
  // want. Measured case: robot nose-in against a dead end, all four frontiers
  // required 135-171 deg. With an absolute penalty the scale no longer
  // decides whether to turn (that was forced) but which frontier to turn to,
  // and since it was also divided by distance it charged 30 units to the one
  // 28 m away and 264 to the 207-cell one 1.1 m in front: raising the scale
  // actively pushed the robot into the long u-turn.
  //
  // Relative, that case collapses to ~0 penalty for everyone and real
  // distance/size decide, while in a corridor with a frontier straight ahead
  // anything behind still pays the full differential. It is also rank-stable:
  // subtracting the minimum shifts every cost by the same constant, so the
  // ordering does not jump when the straightest frontier is consumed.
  std::vector<double> turn_sq(frontiers.size(), 0.0);
  double min_turn_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < frontiers.size(); ++i) {
    // NaN approach_yaw means the frontier is essentially on top of the robot,
    // where no meaningful heading exists: left at zero, and kept out of the
    // minimum so it cannot drag the reference down to zero for everyone.
    if (!std::isfinite(frontiers[i].approach_yaw)) {
      continue;
    }
    double angle_diff = frontiers[i].approach_yaw - yaw;
    angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
    turn_sq[i] = angle_diff * angle_diff;
    min_turn_sq = std::min(min_turn_sq, turn_sq[i]);
  }
  if (!std::isfinite(min_turn_sq)) {
    min_turn_sq = 0.0;
  }

  if (orientation_scale_ > 0.0 || commitment_scale_ > 0.0) {
    for (size_t i = 0; i < frontiers.size(); ++i) {
      auto& f = frontiers[i];
      // turn_sq comes from approach_yaw, the heading of the real
      // (wall-respecting) BFS route, so a frontier that looks straight ahead
      // but is only reachable by backtracking is correctly charged.
      //
      // Squared (not linear) so small/moderate turns barely cost anything
      // while a near-180 degree turn is disproportionately punished -- a
      // linear scale cannot represent that shape. No distance discount: a
      // rotation takes the same time whether the trip is 2 m or 30 m, and
      // dividing by distance is precisely what made far frontiers immune.
      if (orientation_scale_ > 0.0) {
        f.cost += orientation_scale_ * (turn_sq[i] - min_turn_sq);
      }
      if (commitment_scale_ > 0.0 && goal_active_ &&
          same_point(prev_goal_, f.centroid, goal_identity_tolerance_)) {
        // hysteresis is expressed in metres of virtual head start, so it
        // rides on the same scale as progress: a challenger has to be worth
        // more than "commitment_scale_ * commitment_hysteresis_" to win.
        // it erodes if the robot actually loses ground on the goal (negative
        // progress), so an unreachable goal cannot lock the robot in.
        //
        // capped at commitment_max_: uncapped, the bonus grew without limit
        // (measured: 1240 cost units after 8.4 m of progress, i.e. ~1000
        // metres of equivalent immunity) and no frontier, however close or
        // large, could ever take the goal back.
        double progress = committed_goal_initial_distance_ - f.min_distance;
        f.cost -= commitment_scale_ *
                  std::clamp(progress + commitment_hysteresis_, 0.0,
                             commitment_max_);
      }
    }
    std::sort(frontiers.begin(), frontiers.end(),
              [](const frontier_exploration::Frontier& f1,
                 const frontier_exploration::Frontier& f2) {
                return f1.cost < f2.cost;
              });
  }

  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    const auto& f = frontiers[i];
    double turn = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(f.approach_yaw)) {
      turn = std::atan2(std::sin(f.approach_yaw - yaw),
                        std::cos(f.approach_yaw - yaw)) * 180.0 / M_PI;
    }
    RCLCPP_DEBUG(logger_,
                 "frontier %zd cost: %f (size=%u cells, dist=%f m, turn=%.0f "
                 "deg%s)",
                 i, f.cost, f.size, f.min_distance, turn,
                 (goal_active_ &&
                  same_point(prev_goal_, f.centroid, goal_identity_tolerance_))
                     ? ", COMMITTED"
                     : "");
  }

  if (frontiers.empty()) {
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal =
      same_point(prev_goal_, target_position, goal_identity_tolerance_);

  if (!same_goal) {
    // brand new goal: reset the reference distance the commitment bonus
    // grows from.
    committed_goal_initial_distance_ = frontier->min_distance;
  }

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (goal_active_ &&
      (this->now() - last_progress_ >
       tf2::durationFromSec(progress_timeout_)) &&
      !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "Adding current goal to black list");
    makePlan();
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal && goal_active_) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  if (orientation_scale_ > 0.0) {
    // face the robot from its current pose towards the frontier target,
    // so arriving also points the (forward-facing, limited-FOV) sensor
    // into the unknown area instead of leaving an arbitrary heading.
    double dx = target_position.x - pose.position.x;
    double dy = target_position.y - pose.position.y;
    double yaw = std::atan2(dy, dx);
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
  } else {
    goal.pose.pose.orientation.w = 1.;
  }
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  goal_active_ = true;
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Goal was REJECTED by the action server");
          goal_active_ = false;
        } else {
          active_goal_id_ = goal_handle->get_goal_id();
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };

  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  goal_active_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      last_progress_ = this->now();
      prev_distance_ = 0;
      break;
    case rclcpp_action::ResultCode::ABORTED:
#ifdef NAV2_RESULT_HAS_ERROR_CODE
      if (result.result && result.result->error_code != 0) {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=%d (%s) — blacklisting frontier",
                     result.result->error_code,
                     result.result->error_msg.c_str());
        frontier_blacklist_.push_back(frontier_goal);
      } else {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=0 — likely a preemption, not blacklisting");
      }
#else
      // Humble: no error_code field, blacklist unconditionally on abort
      RCLCPP_DEBUG(logger_, "Goal aborted — blacklisting frontier");
      frontier_blacklist_.push_back(frontier_goal);
#endif
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  goal_active_ = false;
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
