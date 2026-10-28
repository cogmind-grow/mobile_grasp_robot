#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using namespace std::placeholders;

class Commander
{
public:
  explicit Commander(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  {
    arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    arm_->setEndEffectorLink("tool_link");
    arm_->setPoseReferenceFrame("base_link");
    arm_->setPlanningTime(10.0);
    arm_->setNumPlanningAttempts(10);
    arm_->setMaxVelocityScalingFactor(0.5);
    arm_->setMaxAccelerationScalingFactor(0.5);

    gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
    gripper_->setMaxVelocityScalingFactor(0.5);
    gripper_->setMaxAccelerationScalingFactor(0.5);

    // 与 worlds/small_warehouse.sdf 中 bottle 摆放一致（相对 base_link）
    bottle_x_ = node_->declare_parameter<double>("bottle_x", 0.55);
    bottle_y_ = node_->declare_parameter<double>("bottle_y", 0.0);
    // 抓取高度：瓶身中上部（世界约 0.12m，base_link 约在轮心高度 0.1m）
    bottle_z_ = node_->declare_parameter<double>("bottle_z", 0.12);
    approach_offset_ = node_->declare_parameter<double>("approach_offset", 0.15);
    lift_offset_ = node_->declare_parameter<double>("lift_offset", 0.12);
    auto_pick_ = node_->declare_parameter<bool>("auto_pick", true);

    attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/growing_bot/bottle/attach", 10);
    detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/growing_bot/bottle/detach", 10);

    open_gripper_sub_ = node_->create_subscription<Bool>(
      "open_gripper", 10, std::bind(&Commander::OpenGripperCallback, this, _1));
    pick_bottle_sub_ = node_->create_subscription<Bool>(
      "pick_bottle", 10, std::bind(&Commander::PickBottleCallback, this, _1));

    if (auto_pick_) {
      // 等 move_group / 控制器就绪后再抓取
      auto_pick_timer_ = node_->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
          auto_pick_timer_->cancel();
          RCLCPP_INFO(node_->get_logger(), "auto_pick: start pickBottle()");
          pickBottle();
        });
    }
  }

  void goToNamedTarget(const std::string & name)
  {
    arm_->setStartStateToCurrentState();
    arm_->setNamedTarget(name);
    planAndExecute(arm_);
  }

  void goToJointTarget(const std::vector<double> & joints)
  {
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(joints);
    planAndExecute(arm_);
  }

  bool goToPoseTarget(
    double x, double y, double z,
    double roll, double pitch, double yaw,
    bool cartesian_path = false)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = node_->now();
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();

    arm_->setStartStateToCurrentState();

    if (!cartesian_path) {
      arm_->setPoseTarget(target_pose);
      return planAndExecute(arm_);
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose.pose);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction =
      arm_->computeCartesianPath(waypoints, 0.01, trajectory, true);

    if (fraction > 0.9) {
      const auto result = arm_->execute(trajectory);
      return result == moveit::core::MoveItErrorCode::SUCCESS;
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Cartesian path only %.1f%% complete, falling back to pose target",
      fraction * 100.0);
    arm_->setPoseTarget(target_pose);
    return planAndExecute(arm_);
  }

  void openGripper()
  {
    gripper_->setStartStateToCurrentState();
    // SRDF: group_state name="open"
    gripper_->setNamedTarget("open");
    planAndExecute(gripper_);
  }

  void closeGripper()
  {
    gripper_->setStartStateToCurrentState();
    // SRDF: group_state name="close"
    gripper_->setNamedTarget("close");
    planAndExecute(gripper_);
  }

  void attachBottle()
  {
    std_msgs::msg::Empty msg;
    attach_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Published bottle attach");
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }

  void detachBottle()
  {
    std_msgs::msg::Empty msg;
    detach_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Published bottle detach");
    rclcpp::sleep_for(std::chrono::milliseconds(300));
  }

  /**
   * 抓取流程：
   * open → approach(上方) → grasp → close → attach → lift
   * 末端朝下 (pitch=pi)，与 bottle (0.55, 0, ~0.09) 对齐
   */
  bool pickBottle()
  {
    // tool_link +Z 朝下，手指沿 X 开合夹住竖直瓶子
    const double roll = 0.0;
    const double pitch = M_PI;
    const double yaw = 0.0;

    const double approach_z = bottle_z_ + approach_offset_;
    const double lift_z = bottle_z_ + lift_offset_;

    RCLCPP_INFO(
      node_->get_logger(),
      "pickBottle: bottle=(%.3f, %.3f, %.3f) approach_z=%.3f",
      bottle_x_, bottle_y_, bottle_z_, approach_z);

    openGripper();

    if (!goToPoseTarget(bottle_x_, bottle_y_, approach_z, roll, pitch, yaw, false)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to reach approach pose");
      return false;
    }

    if (!goToPoseTarget(bottle_x_, bottle_y_, bottle_z_, roll, pitch, yaw, true)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to reach grasp pose");
      return false;
    }

    closeGripper();
    attachBottle();

    if (!goToPoseTarget(bottle_x_, bottle_y_, lift_z, roll, pitch, yaw, true)) {
      RCLCPP_WARN(node_->get_logger(), "Lift after grasp failed (bottle may still be attached)");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "pickBottle: done");
    return true;
  }

private:
  bool planAndExecute(const std::shared_ptr<MoveGroupInterface> & interface)
  {
    // Refresh right before plan: long cartesian/planning can leave a stale start state
    interface->setStartStateToCurrentState();

    MoveGroupInterface::Plan plan;
    const bool success =
      (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed for group [%s]",
        interface->getName().c_str());
      return false;
    }

    const auto result = interface->execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Execution failed for group [%s]",
        interface->getName().c_str());
      return false;
    }
    return true;
  }

  void OpenGripperCallback(const Bool & msg)
  {
    if (msg.data) {
      openGripper();
    } else {
      closeGripper();
    }
  }

  void PickBottleCallback(const Bool & msg)
  {
    if (msg.data) {
      pickBottle();
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;
  std::shared_ptr<MoveGroupInterface> gripper_;
  rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
  rclcpp::Subscription<Bool>::SharedPtr pick_bottle_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  rclcpp::TimerBase::SharedPtr auto_pick_timer_;

  double bottle_x_{0.55};
  double bottle_y_{0.0};
  double bottle_z_{0.12};
  double approach_offset_{0.15};
  double lift_offset_{0.12};
  bool auto_pick_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MoveGroupInterface::plan/execute 需要节点在后台 spin
  auto node = std::make_shared<rclcpp::Node>("commander");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  auto commander = std::make_shared<Commander>(node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
