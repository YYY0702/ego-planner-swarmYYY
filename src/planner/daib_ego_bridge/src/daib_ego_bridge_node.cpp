#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>

namespace daib_ego_bridge
{

class Bridge
{
public:
  Bridge() : private_nh_("~")
  {
    readParameters();
    goal_sub_ = nh_.subscribe(
        explorer_goal_topic_, 1, &Bridge::goalCallback, this);
    generation_sub_ = nh_.subscribe(
        explorer_generation_topic_, 1, &Bridge::generationCallback, this);
    ready_sub_ = nh_.subscribe(
        explorer_ready_topic_, 1, &Bridge::readyCallback, this);
    odom_sub_ = nh_.subscribe(
        odom_topic_, 1, &Bridge::odomCallback, this,
        ros::TransportHints().tcpNoDelay());

    planner_goal_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(planner_goal_topic_, 1, true);
    accepted_generation_pub_ =
        nh_.advertise<std_msgs::UInt64>(accepted_generation_topic_, 1, true);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 1, true);
    watchdog_timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / watchdog_rate_hz_),
        &Bridge::watchdogCallback, this);

    publishState("WAIT_EXPLORER");
    ROS_INFO_STREAM("[ DAIB-EGO Bridge ] goal=" << explorer_goal_topic_
                    << " -> " << planner_goal_topic_
                    << ", odom=" << odom_topic_
                    << ", frame=" << expected_frame_);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber goal_sub_;
  ros::Subscriber generation_sub_;
  ros::Subscriber ready_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher planner_goal_pub_;
  ros::Publisher accepted_generation_pub_;
  ros::Publisher state_pub_;
  ros::WallTimer watchdog_timer_;

  std::string explorer_goal_topic_ = "/daib_explorer/goal";
  std::string explorer_generation_topic_ = "/daib_explorer/generation";
  std::string explorer_ready_topic_ = "/daib_explorer/ready";
  std::string odom_topic_ = "/daib_slam/odom";
  std::string planner_goal_topic_ = "/daib_ego/goal";
  std::string accepted_generation_topic_ =
      "/daib_ego/accepted_generation";
  std::string state_topic_ = "/daib_ego/bridge_state";
  std::string expected_frame_ = "camera_init";
  double max_odom_age_s_ = 1.0;
  double max_ready_age_s_ = 2.5;
  double max_goal_age_s_ = 15.0;
  double max_goal_future_skew_s_ = 0.5;
  double min_goal_distance_m_ = 0.5;
  double max_goal_distance_m_ = 20.0;
  double min_goal_z_m_ = -2.5;
  double max_goal_z_m_ = 4.5;
  double generation_pair_window_s_ = 1.0;
  double watchdog_rate_hz_ = 10.0;

  bool explorer_ready_ = false;
  bool have_odom_ = false;
  bool have_pending_goal_ = false;
  bool have_accepted_goal_ = false;
  nav_msgs::Odometry latest_odom_;
  geometry_msgs::PoseStamped pending_goal_;
  geometry_msgs::PoseStamped accepted_goal_;
  ros::WallTime last_odom_receive_;
  ros::WallTime last_ready_receive_;
  ros::WallTime last_goal_receive_;
  ros::WallTime last_generation_receive_;
  bool have_latest_generation_ = false;
  std::uint64_t latest_generation_ = 0;
  std::uint64_t accepted_generation_ = 0;
  std::string state_;

  void readParameters()
  {
    private_nh_.param(
        "topics/explorer_goal", explorer_goal_topic_, explorer_goal_topic_);
    private_nh_.param("topics/explorer_generation",
                      explorer_generation_topic_,
                      explorer_generation_topic_);
    private_nh_.param(
        "topics/explorer_ready", explorer_ready_topic_, explorer_ready_topic_);
    private_nh_.param("topics/odom", odom_topic_, odom_topic_);
    private_nh_.param(
        "topics/planner_goal", planner_goal_topic_, planner_goal_topic_);
    private_nh_.param("topics/accepted_generation",
                      accepted_generation_topic_,
                      accepted_generation_topic_);
    private_nh_.param("topics/state", state_topic_, state_topic_);
    private_nh_.param("expected_frame", expected_frame_, expected_frame_);
    private_nh_.param("max_odom_age_s", max_odom_age_s_, max_odom_age_s_);
    private_nh_.param(
        "max_ready_age_s", max_ready_age_s_, max_ready_age_s_);
    private_nh_.param("max_goal_age_s", max_goal_age_s_, max_goal_age_s_);
    private_nh_.param("max_goal_future_skew_s",
                      max_goal_future_skew_s_,
                      max_goal_future_skew_s_);
    private_nh_.param(
        "min_goal_distance_m", min_goal_distance_m_, min_goal_distance_m_);
    private_nh_.param(
        "max_goal_distance_m", max_goal_distance_m_, max_goal_distance_m_);
    private_nh_.param("min_goal_z_m", min_goal_z_m_, min_goal_z_m_);
    private_nh_.param("max_goal_z_m", max_goal_z_m_, max_goal_z_m_);
    private_nh_.param("generation_pair_window_s",
                      generation_pair_window_s_,
                      generation_pair_window_s_);
    private_nh_.param(
        "watchdog_rate_hz", watchdog_rate_hz_, watchdog_rate_hz_);

    max_odom_age_s_ = std::max(0.1, max_odom_age_s_);
    max_ready_age_s_ = std::max(0.1, max_ready_age_s_);
    max_goal_age_s_ = std::max(0.0, max_goal_age_s_);
    max_goal_future_skew_s_ = std::max(0.0, max_goal_future_skew_s_);
    min_goal_distance_m_ = std::max(0.0, min_goal_distance_m_);
    max_goal_distance_m_ =
        std::max(min_goal_distance_m_, max_goal_distance_m_);
    if (min_goal_z_m_ > max_goal_z_m_)
      std::swap(min_goal_z_m_, max_goal_z_m_);
    generation_pair_window_s_ = std::max(0.05, generation_pair_window_s_);
    watchdog_rate_hz_ = std::max(1.0, watchdog_rate_hz_);
  }

  void publishState(const std::string &state)
  {
    if (state == state_) return;
    state_ = state;
    std_msgs::String message;
    message.data = state;
    state_pub_.publish(message);
    ROS_INFO_STREAM("[ DAIB-EGO Bridge ] state=" << state);
  }

  static double distance(const geometry_msgs::Point &a,
                         const geometry_msgs::Point &b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  bool odomFresh() const
  {
    return have_odom_ &&
           (ros::WallTime::now() - last_odom_receive_).toSec() <=
               max_odom_age_s_;
  }

  bool explorerFresh() const
  {
    return explorer_ready_ &&
           (ros::WallTime::now() - last_ready_receive_).toSec() <=
               max_ready_age_s_;
  }

  bool sameGoalIdentity(const geometry_msgs::PoseStamped &left,
                        const geometry_msgs::PoseStamped &right) const
  {
    if (left.header.frame_id != right.header.frame_id) return false;
    const bool both_stamped =
        !left.header.stamp.isZero() && !right.header.stamp.isZero();
    if (both_stamped && left.header.stamp != right.header.stamp) return false;
    if (!both_stamped &&
        (left.header.stamp.isZero() != right.header.stamp.isZero()))
      return false;
    return distance(left.pose.position, right.pose.position) <= 1e-3;
  }

  bool validatePending(std::string &reason) const
  {
    const geometry_msgs::PoseStamped &goal = pending_goal_;
    const geometry_msgs::Point &point = goal.pose.position;
    if (!explorerFresh())
    {
      reason = "WAIT_EXPLORER";
      return false;
    }
    if (!odomFresh())
    {
      reason = "WAIT_ODOM";
      return false;
    }
    if (!expected_frame_.empty() &&
        (goal.header.frame_id != expected_frame_ ||
         latest_odom_.header.frame_id != expected_frame_))
    {
      reason = "REJECT_FRAME";
      return false;
    }
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) ||
        point.z < min_goal_z_m_ || point.z > max_goal_z_m_)
    {
      reason = "REJECT_BOUNDS";
      return false;
    }
    if (max_goal_age_s_ > 0.0)
    {
      double age = 0.0;
      const bool use_sensor_time =
          !goal.header.stamp.isZero() &&
          !latest_odom_.header.stamp.isZero();
      if (use_sensor_time)
      {
        // The DAIB odometry and planning cloud are published from the same
        // LIO update and therefore share a clock domain. ros::Time::now()
        // may instead follow bag /clock or wall time, so it must not be mixed
        // with the sensor-domain goal stamp.
        age = (latest_odom_.header.stamp - goal.header.stamp).toSec();
        if (age < -max_goal_future_skew_s_)
        {
          reason = "WAIT_ODOM_SYNC";
          ROS_WARN_STREAM_THROTTLE(
              2.0, "[ DAIB-EGO Bridge ] goal is "
                       << -age << " s ahead of odometry; waiting for sync");
          return false;
        }
      }
      else
      {
        age = (ros::WallTime::now() - last_goal_receive_).toSec();
      }
      if (age > max_goal_age_s_)
      {
        reason = "REJECT_STALE";
        ROS_WARN_STREAM_THROTTLE(
            2.0, "[ DAIB-EGO Bridge ] reject stale goal: age="
                     << age << " s, source="
                     << (use_sensor_time ? "odom_stamp" : "wall_receive"));
        return false;
      }
    }
    const double goal_distance =
        distance(point, latest_odom_.pose.pose.position);
    if (goal_distance < min_goal_distance_m_ ||
        goal_distance > max_goal_distance_m_)
    {
      reason = "REJECT_DISTANCE";
      return false;
    }
    if (have_accepted_goal_ && sameGoalIdentity(goal, accepted_goal_))
    {
      reason = "DUPLICATE_GOAL";
      return false;
    }
    return true;
  }

  void publishAcceptedGeneration()
  {
    std_msgs::UInt64 generation;
    generation.data = accepted_generation_;
    accepted_generation_pub_.publish(generation);
  }

  void tryForward()
  {
    if (!have_pending_goal_) return;
    std::string reason;
    if (!validatePending(reason))
    {
      publishState(reason);
      if (reason.find("REJECT_") == 0 ||
          reason.find("DUPLICATE_") == 0)
        have_pending_goal_ = false;
      return;
    }

    accepted_goal_ = pending_goal_;
    const bool generation_matches_goal =
        have_latest_generation_ && !last_goal_receive_.isZero() &&
        !last_generation_receive_.isZero() &&
        std::fabs((last_generation_receive_ - last_goal_receive_).toSec()) <=
            generation_pair_window_s_;
    if (generation_matches_goal &&
        latest_generation_ > accepted_generation_)
      accepted_generation_ = latest_generation_;
    else
      ++accepted_generation_;
    have_accepted_goal_ = true;
    have_pending_goal_ = false;
    planner_goal_pub_.publish(accepted_goal_);
    publishAcceptedGeneration();
    publishState("GOAL_FORWARDED");
    ROS_INFO_STREAM("[ DAIB-EGO Bridge ] generation="
                    << accepted_generation_ << ", goal=("
                    << accepted_goal_.pose.position.x << ", "
                    << accepted_goal_.pose.position.y << ", "
                    << accepted_goal_.pose.position.z << ")");
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr &message)
  {
    pending_goal_ = *message;
    last_goal_receive_ = ros::WallTime::now();
    have_pending_goal_ = true;
    tryForward();
  }

  void generationCallback(const std_msgs::UInt64ConstPtr &message)
  {
    if (message->data == 0) return;
    latest_generation_ = message->data;
    have_latest_generation_ = true;
    last_generation_receive_ = ros::WallTime::now();
    const bool follows_accepted_goal =
        have_accepted_goal_ && !last_goal_receive_.isZero() &&
        (last_generation_receive_ - last_goal_receive_).toSec() >= 0.0 &&
        (last_generation_receive_ - last_goal_receive_).toSec() <=
            generation_pair_window_s_;
    if (follows_accepted_goal &&
        latest_generation_ > accepted_generation_)
    {
      accepted_generation_ = latest_generation_;
      publishAcceptedGeneration();
    }
  }

  void readyCallback(const std_msgs::BoolConstPtr &message)
  {
    explorer_ready_ = message->data;
    last_ready_receive_ = ros::WallTime::now();
    if (!explorer_ready_) publishState("WAIT_EXPLORER");
    else tryForward();
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &message)
  {
    latest_odom_ = *message;
    have_odom_ = true;
    last_odom_receive_ = ros::WallTime::now();
    tryForward();
  }

  void watchdogCallback(const ros::WallTimerEvent &)
  {
    if (!explorerFresh()) publishState("WAIT_EXPLORER");
    else if (!odomFresh()) publishState("WAIT_ODOM");
    else if (have_pending_goal_) tryForward();
  }
};

}  // namespace daib_ego_bridge

int main(int argc, char **argv)
{
  ros::init(argc, argv, "daib_ego_bridge");
  daib_ego_bridge::Bridge bridge;
  ros::spin();
  return 0;
}
