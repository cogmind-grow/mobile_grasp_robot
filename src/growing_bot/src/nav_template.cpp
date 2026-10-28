#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class NavTemplate : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  NavTemplate()
  : Node("nav_template")
  {
    initial_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .transient_local();
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", amcl_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
        initial_pose_received_ = true;
      });

    follow_waypoints_client_ =
      rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
  }

  bool waitUntilNav2Active()
  {
    RCLCPP_INFO(get_logger(), "Waiting for the Nav2 FollowWaypoints action server...");
    while (rclcpp::ok() && !follow_waypoints_client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(get_logger(), "Nav2 is not active yet");
    }
    return rclcpp::ok();
  }

  bool setInitialPoseAndWait(double x, double y, double yaw)
  {
    constexpr double pi = 3.14159265358979323846;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.pose.pose = createPose(x, y, yaw).pose;

    // 给 AMCL 一个合理的初始不确定度。
    initial_pose.pose.covariance[0] = 0.25;
    initial_pose.pose.covariance[7] = 0.25;
    initial_pose.pose.covariance[35] = std::pow(15.0 * pi / 180.0, 2);

    RCLCPP_INFO(get_logger(), "Publishing initial pose and waiting for AMCL...");
    while (rclcpp::ok() && !initial_pose_received_) {
      initial_pose.header.stamp = now();
      initial_pose_pub_->publish(initial_pose);
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(1s);
    }

    return rclcpp::ok();
  }

  void followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> & waypoints)
  {
    FollowWaypoints::Goal goal;
    goal.poses = waypoints;

    auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    options.goal_response_callback =
      [this](const GoalHandleFollowWaypoints::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "FollowWaypoints goal was rejected");
          task_complete_ = true;
          return;
        }
        RCLCPP_INFO(get_logger(), "FollowWaypoints goal accepted");
      };
    options.feedback_callback =
      [this](
      GoalHandleFollowWaypoints::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        RCLCPP_INFO(
          get_logger(), "Heading to waypoint %u",
          static_cast<unsigned int>(feedback->current_waypoint + 1));
      };
    options.result_callback =
      [this](const GoalHandleFollowWaypoints::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "All waypoints completed");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Waypoint navigation was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Waypoint navigation was canceled");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Waypoint navigation returned an unknown result");
            break;
        }
        task_complete_ = true;
      };

    task_complete_ = false;
    follow_waypoints_client_->async_send_goal(goal, options);

    while (rclcpp::ok() && !task_complete_) {
      rclcpp::spin_some(shared_from_this());
      rclcpp::sleep_for(100ms);
    }
  }

  geometry_msgs::msg::PoseStamped createPose(double x, double y, double yaw)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    return pose;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
  bool initial_pose_received_{false};
  bool task_complete_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto navigator = std::make_shared<NavTemplate>();

  if (!navigator->waitUntilNav2Active() ||
    !navigator->setInitialPoseAndWait(0.0, 0.0, 0.0))
  {
    rclcpp::shutdown();
    return 1;
  }

  std::vector<geometry_msgs::msg::PoseStamped> waypoints{
    navigator->createPose(1.0, 0.0, 0.0)
  };
  navigator->followWaypoints(waypoints);

  rclcpp::shutdown();
  return 0;
}
