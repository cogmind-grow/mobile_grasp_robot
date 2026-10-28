#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class CmdVelStamper : public rclcpp::Node
{
public:
  CmdVelStamper()
  : Node("cmd_vel_stamper")
  {
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "diff_drive_controller/cmd_vel", 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = now();
        stamped.header.frame_id = "base_link";
        stamped.twist = *msg;
        publisher_->publish(stamped);
      });
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelStamper>());
  rclcpp::shutdown();
  return 0;
}
