// 瓶子抓取状态机（节点: pick_state_machine）
//
// 状态流：IDLE → SEARCH_BOTTLE → BOTTLE_FOUND → MOVE_TO_BOTTLE → GRASP → ATTACH → DONE
//
// - 瓶子检测：订阅 detector/bottle_localizer.py 发布的 /growing_bot/bottle/position
//   （瓶子 XYZ，base_link 坐标系）
// - 可达判断：BOTTLE_FOUND 时检查瓶子是否在机械臂工作空间内
//   - 可达   → 直接 GRASP
//   - 不可达 → MOVE_TO_BOTTLE（Nav2）后回到 SEARCH_BOTTLE 重新检测
// - 底盘移动：Nav2 FollowWaypoints action（参考 nav_template.cpp）
// - 机械臂：MoveIt MoveGroupInterface（参考 moveit_template.cpp）
// - ATTACH：发布 /growing_bot/bottle/attach，gz DetachableJoint 把瓶子焊到夹爪
//
// 当前状态发布在 /growing_bot/pick_state（std_msgs/String），可实时监控。

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

enum class State
{
  IDLE,
  SEARCH_BOTTLE,
  BOTTLE_FOUND,
  MOVE_TO_BOTTLE,
  GRASP,
  ATTACH,
  DONE,
};

static const char * stateName(State state)
{
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::SEARCH_BOTTLE: return "SEARCH_BOTTLE";
    case State::BOTTLE_FOUND: return "BOTTLE_FOUND";
    case State::MOVE_TO_BOTTLE: return "MOVE_TO_BOTTLE";
    case State::GRASP: return "GRASP";
    case State::ATTACH: return "ATTACH";
    case State::DONE: return "DONE";
    default: return "UNKNOWN";
  }
}

class PickManager
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  explicit PickManager(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  {
    // ---- 参数 ----
    // IDLE：是否发布初始位姿并等待 AMCL（参考 nav_template.cpp）
    set_initial_pose_ = node_->declare_parameter<bool>("set_initial_pose", true);
    initial_pose_x_ = node_->declare_parameter<double>("initial_pose_x", 0.0);
    initial_pose_y_ = node_->declare_parameter<double>("initial_pose_y", 0.0);
    initial_pose_yaw_ = node_->declare_parameter<double>("initial_pose_yaw", 0.0);

    // 可达性判断（相对 base_link）：水平距离区间 + 最大抓取高度
    min_reach_ = node_->declare_parameter<double>("min_reach", 0.25);
    max_reach_ = node_->declare_parameter<double>("max_reach", 0.65);
    max_grasp_z_ = node_->declare_parameter<double>("max_grasp_z", 0.45);

    // 检测点是相机射线命中的瓶身正面，沿射线方向补偿半径得到瓶身中心
    bottle_radius_ = node_->declare_parameter<double>("bottle_radius", 0.029);

    // 抓取（参考 moveit_template.cpp 的抓取流程）
    grasp_z_ = node_->declare_parameter<double>("grasp_z", 0.12);
    approach_offset_ = node_->declare_parameter<double>("approach_offset", 0.15);
    lift_offset_ = node_->declare_parameter<double>("lift_offset", 0.12);

    // MOVE_TO_BOTTLE：导航到距瓶子 nav_standoff_ 米处（正对瓶子）
    // 注意与 nav2_params.yaml 的 xy_goal_tolerance 耦合：实际停点距瓶子
    // ∈ [nav_standoff − tol, nav_standoff + tol]，必须落在可达窗 [min_reach, max_reach] 内
    nav_standoff_ = node_->declare_parameter<double>("nav_standoff", 0.5);
    max_nav_attempts_ = node_->declare_parameter<int>("max_nav_attempts", 2);
    nav_timeout_ = node_->declare_parameter<double>("nav_timeout", 300.0);

    // ---- MoveIt（参考 moveit_template.cpp）----
    RCLCPP_INFO(node_->get_logger(), "Waiting for MoveGroup (move_group) ...");
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

    // ---- attach / detach（gz DetachableJoint）----
    attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/growing_bot/bottle/attach", 10);
    detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/growing_bot/bottle/detach", 10);

    // ---- Nav2 action client（参考 nav_template.cpp）----
    follow_waypoints_client_ =
      rclcpp_action::create_client<FollowWaypoints>(node_, "follow_waypoints");

    // ---- 初始位姿（AMCL）----
    initial_pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10);
    auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .transient_local();
    amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", amcl_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
        initial_pose_received_ = true;
      });

    // ---- 瓶子位置（bottle_localizer.py 发布，base_link 坐标系）----
    bottle_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/growing_bot/bottle/position", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        bottleCallback(msg);
      });

    // ---- 状态发布（ros2 topic echo /growing_bot/pick_state 可监控）----
    state_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/growing_bot/pick_state", 10);

    // ---- TF2：base_link → map 需要 AMCL，用于把瓶子变换到 map 系算导航目标 ----
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_);
  }

  // 状态机主循环（在独立线程运行，node 由主线程的 executor spin）
  void run()
  {
    setState(State::IDLE);

    if (!waitUntilNav2Active()) {
      finish(false, "nav2 action server unavailable");
    } else if (set_initial_pose_ && !setInitialPoseAndWait()) {
      finish(false, "interrupted while waiting for AMCL");
    }

    while (rclcpp::ok() && !done_) {
      switch (state_) {
        case State::IDLE:
          setState(State::SEARCH_BOTTLE);
          break;

        case State::SEARCH_BOTTLE:
          {
            // 等待新的检测结果；底盘移动后瓶子在 base_link 下的坐标已变化，
            // 必须重新检测，不能复用旧数据
            if (!waitForBottlePose()) {
              finish(false, "interrupted while searching bottle");
              break;
            }
            setState(State::BOTTLE_FOUND);
          }
          break;

        case State::BOTTLE_FOUND:
          {
            double cx, cy, cz;
            getBottleCenter(cx, cy, cz);
            RCLCPP_INFO(node_->get_logger(),
              "BOTTLE_FOUND: bottle center XYZ(base_link) = (%.3f, %.3f, %.3f)",
              cx, cy, cz);

            const double dist = std::hypot(cx, cy);
            if (dist >= min_reach_ && dist <= max_reach_ && cz <= max_grasp_z_) {
              RCLCPP_INFO(node_->get_logger(),
                "BOTTLE_FOUND: bottle within arm reach (dist=%.3f), grasp directly",
                dist);
              setState(State::GRASP);
            } else if (nav_attempts_ < max_nav_attempts_) {
              RCLCPP_INFO(node_->get_logger(),
                "BOTTLE_FOUND: bottle out of arm reach (dist=%.3f, "
                "require [%.2f, %.2f], z<=%.2f), move base first",
                dist, min_reach_, max_reach_, max_grasp_z_);
              setState(State::MOVE_TO_BOTTLE);
            } else {
              finish(false, "bottle still unreachable after nav attempts (dist=" +
                std::to_string(dist) + ")");
            }
          }
          break;

        case State::MOVE_TO_BOTTLE:
          {
            nav_attempts_++;
            if (moveToBottle()) {
              // 底盘已就位，回到 SEARCH_BOTTLE 重新检测
              setState(State::SEARCH_BOTTLE);
            } else if (nav_attempts_ >= max_nav_attempts_) {
              finish(false, "nav to bottle failed");
            } else {
              RCLCPP_WARN(node_->get_logger(),
                "MOVE_TO_BOTTLE failed, re-detect and retry");
              setState(State::SEARCH_BOTTLE);
            }
          }
          break;

        case State::GRASP:
          {
            double cx, cy, cz;
            getBottleCenter(cx, cy, cz);
            if (graspBottle(cx, cy)) {
              setState(State::ATTACH);
            } else {
              finish(false, "grasp failed");
            }
          }
          break;

        case State::ATTACH:
          {
            double cx, cy, cz;
            getBottleCenter(cx, cy, cz);
            attachAndLift(cx, cy);
            finish(true, "bottle picked and attached");
          }
          break;

        case State::DONE:
          break;
      }
    }

    // 结束本进程（让主线程 executor.spin() 返回）
    rclcpp::shutdown();
  }

private:
  // ================= 状态机辅助 =================

  void setState(State state)
  {
    state_ = state;
    std_msgs::msg::String msg;
    msg.data = stateName(state);
    state_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "=== FSM: %s ===", msg.data.c_str());
  }

  void finish(bool success, const std::string & message)
  {
    done_ = true;
    success_ = success;
    setState(State::DONE);
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "DONE: %s", message.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "DONE (failed): %s", message.c_str());
    }
  }

  // 等待新的检测结果（进入时复位 have_bottle_，旧数据自然丢弃）
  bool waitForBottlePose()
  {
    RCLCPP_INFO(node_->get_logger(), "SEARCH_BOTTLE: waiting for bottle detection ...");
    std::unique_lock<std::mutex> lock(bottle_mutex_);
    have_bottle_ = false;
    while (rclcpp::ok() && !have_bottle_) {
      if (!bottle_cv_.wait_for(lock, 5s, [this] { return have_bottle_; })) {
        RCLCPP_INFO(node_->get_logger(), "SEARCH_BOTTLE: no bottle detected yet ...");
      }
    }
    return rclcpp::ok();
  }

  void bottleCallback(const geometry_msgs::msg::PointStamped::SharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(bottle_mutex_);
    // 只在 SEARCH_BOTTLE 状态接收第一条新检测；waitForBottlePose() 进入时
    // 已复位 have_bottle_，因此底盘移动前的旧检测不会被使用
    if (state_ != State::SEARCH_BOTTLE || have_bottle_) {
      return;
    }
    bottle_pose_ = *msg;
    have_bottle_ = true;
    bottle_cv_.notify_all();
  }

  void getBottlePose(double & x, double & y, double & z)
  {
    std::lock_guard<std::mutex> lock(bottle_mutex_);
    x = bottle_pose_.point.x;
    y = bottle_pose_.point.y;
    z = bottle_pose_.point.z;
  }

  // 检测点在瓶身正面，沿 机器人→瓶子 方向补偿一个半径得到瓶身中心
  void getBottleCenter(double & x, double & y, double & z)
  {
    getBottlePose(x, y, z);
    const double dist = std::hypot(x, y);
    if (dist > 1e-6) {
      x += x / dist * bottle_radius_;
      y += y / dist * bottle_radius_;
    }
  }

  // ================= 底盘移动（参考 nav_template.cpp）=================

  bool waitUntilNav2Active()
  {
    RCLCPP_INFO(node_->get_logger(),
      "IDLE: waiting for the Nav2 FollowWaypoints action server...");
    while (rclcpp::ok() && !follow_waypoints_client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(node_->get_logger(), "IDLE: Nav2 is not active yet");
    }
    return rclcpp::ok();
  }

  bool setInitialPoseAndWait()
  {
    constexpr double pi = 3.14159265358979323846;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, initial_pose_yaw_);
    initial_pose.pose.pose.position.x = initial_pose_x_;
    initial_pose.pose.pose.position.y = initial_pose_y_;
    initial_pose.pose.pose.orientation.x = quaternion.x();
    initial_pose.pose.pose.orientation.y = quaternion.y();
    initial_pose.pose.pose.orientation.z = quaternion.z();
    initial_pose.pose.pose.orientation.w = quaternion.w();

    // 给 AMCL 一个合理的初始不确定度
    initial_pose.pose.covariance[0] = 0.25;
    initial_pose.pose.covariance[7] = 0.25;
    initial_pose.pose.covariance[35] = std::pow(15.0 * pi / 180.0, 2);

    RCLCPP_INFO(node_->get_logger(),
      "IDLE: publishing initial pose and waiting for AMCL...");
    while (rclcpp::ok() && !initial_pose_received_) {
      initial_pose.header.stamp = node_->now();
      initial_pose_pub_->publish(initial_pose);
      rclcpp::sleep_for(1s);
    }
    return rclcpp::ok();
  }

  // 导航到"距瓶子 nav_standoff_ 米、正对瓶子"处
  bool moveToBottle()
  {
    double bx, by, bz;
    getBottlePose(bx, by, bz);

    // 1. TF: map -> base_link（依赖 AMCL 已定位）
    geometry_msgs::msg::TransformStamped map_to_base;
    bool have_tf = false;
    for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
      try {
        map_to_base = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        have_tf = true;
        break;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(node_->get_logger(),
          "MOVE_TO_BOTTLE: waiting for TF map->base_link: %s", e.what());
        rclcpp::sleep_for(500ms);
      }
    }
    if (!have_tf) {
      RCLCPP_ERROR(node_->get_logger(),
        "MOVE_TO_BOTTLE: TF map->base_link not available");
      return false;
    }

    // 2. 瓶子坐标 base_link -> map
    geometry_msgs::msg::PointStamped bottle_base;
    bottle_base.header.frame_id = "base_link";
    bottle_base.header.stamp = node_->now();
    bottle_base.point.x = bx;
    bottle_base.point.y = by;
    bottle_base.point.z = bz;

    geometry_msgs::msg::PointStamped bottle_map;
    tf2::doTransform(bottle_base, bottle_map, map_to_base);

    // 3. 目标位姿：在 机器人->瓶子 连线上后退 nav_standoff_，并正对瓶子
    const double rx = map_to_base.transform.translation.x;
    const double ry = map_to_base.transform.translation.y;
    const double dx = bottle_map.point.x - rx;
    const double dy = bottle_map.point.y - ry;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) {
      RCLCPP_ERROR(node_->get_logger(),
        "MOVE_TO_BOTTLE: bottle is at the robot position, cannot compute direction");
      return false;
    }
    const double ux = dx / dist;
    const double uy = dy / dist;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = node_->now();
    goal.pose.position.x = bottle_map.point.x - ux * nav_standoff_;
    goal.pose.position.y = bottle_map.point.y - uy * nav_standoff_;
    goal.pose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, std::atan2(uy, ux));
    goal.pose.orientation.x = quaternion.x();
    goal.pose.orientation.y = quaternion.y();
    goal.pose.orientation.z = quaternion.z();
    goal.pose.orientation.w = quaternion.w();

    RCLCPP_INFO(node_->get_logger(),
      "MOVE_TO_BOTTLE: bottle(map) = (%.2f, %.2f), nav goal = (%.2f, %.2f)",
      bottle_map.point.x, bottle_map.point.y,
      goal.pose.position.x, goal.pose.position.y);

    // 4. FollowWaypoints 导航（参考 nav_template.cpp）
    FollowWaypoints::Goal goal_msg;
    goal_msg.poses = {goal};

    auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    options.goal_response_callback =
      [this](const GoalHandleFollowWaypoints::SharedPtr & goal_handle) {
        std::lock_guard<std::mutex> lock(nav_mutex_);
        nav_goal_handle_ = goal_handle;
        if (!goal_handle) {
          RCLCPP_ERROR(node_->get_logger(), "MOVE_TO_BOTTLE: goal was rejected");
          nav_result_ready_ = true;
          nav_succeeded_ = false;
          nav_cv_.notify_all();
          return;
        }
        RCLCPP_INFO(node_->get_logger(), "MOVE_TO_BOTTLE: goal accepted");
      };
    options.feedback_callback =
      [this](
      GoalHandleFollowWaypoints::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        // waypoint_follower 按 loop_rate(20Hz) 发布 feedback，节流到 1 秒一条
        const auto now = std::chrono::steady_clock::now();
        if (now - last_nav_feedback_log_ < std::chrono::seconds(1)) {
          return;
        }
        last_nav_feedback_log_ = now;
        RCLCPP_INFO(node_->get_logger(), "MOVE_TO_BOTTLE: heading to waypoint %u",
          static_cast<unsigned int>(feedback->current_waypoint + 1));
      };
    options.result_callback =
      [this](const GoalHandleFollowWaypoints::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(nav_mutex_);
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(),
              "MOVE_TO_BOTTLE: nav goal reached (standoff point, not the bottle)");
            nav_succeeded_ = true;
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "MOVE_TO_BOTTLE: navigation aborted");
            nav_succeeded_ = false;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "MOVE_TO_BOTTLE: navigation canceled");
            nav_succeeded_ = false;
            break;
          default:
            RCLCPP_ERROR(node_->get_logger(),
              "MOVE_TO_BOTTLE: unknown navigation result");
            nav_succeeded_ = false;
            break;
        }
        nav_result_ready_ = true;
        nav_cv_.notify_all();
      };

    {
      std::lock_guard<std::mutex> lock(nav_mutex_);
      nav_result_ready_ = false;
      nav_succeeded_ = false;
      nav_goal_handle_ = nullptr;
    }
    follow_waypoints_client_->async_send_goal(goal_msg, options);

    // 5. 分片等待结果（保证 Ctrl+C 可退出）；超时则取消目标
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(nav_timeout_));
    {
      std::unique_lock<std::mutex> lock(nav_mutex_);
      while (rclcpp::ok() && !nav_result_ready_) {
        if (nav_cv_.wait_until(lock, std::chrono::steady_clock::now() + 1s) ==
          std::cv_status::timeout &&
          std::chrono::steady_clock::now() >= deadline)
        {
          break;
        }
      }
      if (nav_result_ready_) {
        return nav_succeeded_;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
      "MOVE_TO_BOTTLE: navigation timeout/interrupted, canceling goal");
    GoalHandleFollowWaypoints::SharedPtr goal_handle;
    {
      std::lock_guard<std::mutex> lock(nav_mutex_);
      goal_handle = nav_goal_handle_;
    }
    if (goal_handle) {
      follow_waypoints_client_->async_cancel_goal(goal_handle);
    }
    return false;
  }

  // ================= 机械臂（参考 moveit_template.cpp）=================

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

    RCLCPP_WARN(node_->get_logger(),
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
   * GRASP：open → approach(上方) → grasp → close
   * 末端朝下 (pitch=pi)，手指沿 X 开合夹住竖直瓶子（与 moveit_template 一致）
   */
  bool graspBottle(double bottle_x, double bottle_y)
  {
    const double roll = 0.0;
    const double pitch = M_PI;
    const double yaw = 0.0;

    const double approach_z = grasp_z_ + approach_offset_;

    RCLCPP_INFO(node_->get_logger(),
      "GRASP: bottle=(%.3f, %.3f) grasp_z=%.3f approach_z=%.3f",
      bottle_x, bottle_y, grasp_z_, approach_z);

    openGripper();

    if (!goToPoseTarget(bottle_x, bottle_y, approach_z, roll, pitch, yaw, false)) {
      RCLCPP_ERROR(node_->get_logger(), "GRASP: failed to reach approach pose");
      return false;
    }

    if (!goToPoseTarget(bottle_x, bottle_y, grasp_z_, roll, pitch, yaw, true)) {
      RCLCPP_ERROR(node_->get_logger(), "GRASP: failed to reach grasp pose");
      return false;
    }

    closeGripper();
    RCLCPP_INFO(node_->get_logger(), "GRASP: gripper closed");
    return true;
  }

  // ATTACH：发布 attach 焊接瓶子到夹爪，再抬起
  void attachAndLift(double bottle_x, double bottle_y)
  {
    attachBottle();

    const double lift_z = grasp_z_ + lift_offset_;
    if (!goToPoseTarget(bottle_x, bottle_y, lift_z, 0.0, M_PI, 0.0, true)) {
      RCLCPP_WARN(node_->get_logger(),
        "ATTACH: lift failed (bottle may still be attached)");
    }
  }

  // ================= 成员 =================

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;
  std::shared_ptr<MoveGroupInterface> gripper_;

  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    amcl_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr bottle_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::atomic<State> state_{State::IDLE};
  std::atomic<bool> initial_pose_received_{false};
  bool done_{false};
  bool success_{false};
  int nav_attempts_{0};

  // 瓶子检测结果（由 bottle_mutex_ 保护）
  std::mutex bottle_mutex_;
  std::condition_variable bottle_cv_;
  geometry_msgs::msg::PointStamped bottle_pose_;
  bool have_bottle_{false};

  // 导航结果（由 nav_mutex_ 保护）
  std::mutex nav_mutex_;
  std::condition_variable nav_cv_;
  bool nav_result_ready_{false};
  bool nav_succeeded_{false};
  GoalHandleFollowWaypoints::SharedPtr nav_goal_handle_;
  std::chrono::steady_clock::time_point last_nav_feedback_log_{};

  // 参数
  bool set_initial_pose_{true};
  double initial_pose_x_{0.0};
  double initial_pose_y_{0.0};
  double initial_pose_yaw_{0.0};
  double min_reach_{0.25};
  double max_reach_{0.65};
  double max_grasp_z_{0.45};
  double bottle_radius_{0.029};
  double grasp_z_{0.12};
  double approach_offset_{0.15};
  double lift_offset_{0.12};
  double nav_standoff_{0.5};
  int max_nav_attempts_{2};
  double nav_timeout_{300.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MoveGroupInterface 的 plan/execute 需要节点在后台 spin
  auto node = std::make_shared<rclcpp::Node>("pick_state_machine");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // 构造 PickManager 时会阻塞等待 move_group 就绪；
  // move_group 尚未启动时可能抛异常，重试直到就绪
  std::shared_ptr<PickManager> manager;
  while (rclcpp::ok() && !manager) {
    try {
      manager = std::make_shared<PickManager>(node);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(),
        "Waiting for move_group: %s (retry in 2s)", e.what());
      rclcpp::sleep_for(2s);
    }
  }
  if (!manager) {
    rclcpp::shutdown();
    return 1;
  }

  // 状态机在独立线程运行；主线程 spin 保证
  // MoveIt action、Nav2 action、TF、话题回调都能被处理
  std::thread fsm_thread([manager]() { manager->run(); });

  executor.spin();

  if (fsm_thread.joinable()) {
    fsm_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
