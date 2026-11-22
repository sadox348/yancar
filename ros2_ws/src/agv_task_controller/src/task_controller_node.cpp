#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "agv_task_controller/controller_core.hpp"

using namespace std::chrono_literals;

namespace agv_task_controller
{

class TaskController : public rclcpp::Node
{
public:
  TaskController()
  : Node("agv_task_controller")
  {
    goal_frame_ = declare_parameter("goal_frame", "odom");
    limits_.max_linear = declare_parameter("max_linear", 0.25);
    limits_.max_angular = declare_parameter("max_angular", 0.8);
    limits_.linear_gain = declare_parameter("linear_gain", 0.8);
    limits_.angular_gain = declare_parameter("angular_gain", 1.8);
    limits_.distance_tolerance = declare_parameter("distance_tolerance", 0.05);
    limits_.heading_tolerance = declare_parameter("heading_tolerance", 0.15);
    const double control_rate = declare_parameter("control_rate", 10.0);
    odom_timeout_ = declare_parameter("odom_timeout", 0.5);

    if (goal_frame_.empty()) {
      throw std::invalid_argument("goal_frame must not be empty");
    }
    if (control_rate <= 0.0 || odom_timeout_ <= 0.0 || limits_.max_linear <= 0.0 ||
      limits_.max_angular <= 0.0 || limits_.distance_tolerance < 0.0 ||
      limits_.heading_tolerance < 0.0)
    {
      throw std::invalid_argument("controller rates, limits and tolerances must be valid");
    }

    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_qos);
    state_pub_ = create_publisher<std_msgs::msg::String>("/agv_task/state", state_qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", odom_qos,
      std::bind(&TaskController::odom_callback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", goal_qos,
      std::bind(&TaskController::goal_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate),
      std::bind(&TaskController::control_tick, this));

    publish_state("WAITING_FOR_GOAL");
    RCLCPP_INFO(get_logger(), "Waiting for a /goal_pose in frame '%s'", goal_frame_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != goal_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignored odometry in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), goal_frame_.c_str());
      return;
    }
    const auto & q = msg->pose.pose.orientation;
    const double quaternion_norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!is_finite_goal(msg->pose.pose.position.x, msg->pose.pose.position.y) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w) || quaternion_norm_squared < 1e-12)
    {
      RCLCPP_WARN(get_logger(), "Ignored odometry containing invalid pose values");
      return;
    }
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_odom_time_ = now();
    odom_received_ = true;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != goal_frame_) {
      RCLCPP_WARN(
        get_logger(), "Rejected goal in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), goal_frame_.c_str());
      return;
    }
    if (!is_finite_goal(msg->pose.position.x, msg->pose.position.y)) {
      RCLCPP_WARN(get_logger(), "Rejected goal containing non-finite coordinates");
      return;
    }
    target_x_ = msg->pose.position.x;
    target_y_ = msg->pose.position.y;
    goal_received_ = true;
    goal_reached_ = false;
    RCLCPP_INFO(get_logger(), "Accepted goal (%.3f, %.3f)", target_x_, target_y_);
    publish_state("NEW_GOAL");
  }

  void control_tick()
  {
    if (!goal_received_) {
      publish_stop_and_state("WAITING_FOR_GOAL");
      return;
    }
    if (!odom_received_) {
      publish_stop_and_state("WAITING_FOR_ODOM");
      return;
    }
    if ((now() - last_odom_time_).seconds() > odom_timeout_) {
      publish_stop_and_state("ODOM_TIMEOUT");
      return;
    }

    const auto command = compute_command(x_, y_, yaw_, target_x_, target_y_, limits_);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.linear;
    twist.angular.z = command.angular;
    cmd_pub_->publish(twist);

    if (command.state == MotionState::reached) {
      if (!goal_reached_) {
        RCLCPP_INFO(get_logger(), "Goal reached at (%.3f, %.3f)", x_, y_);
      }
      goal_reached_ = true;
      publish_state("GOAL_REACHED");
    } else if (command.state == MotionState::rotate) {
      goal_reached_ = false;
      publish_state("ROTATING");
    } else {
      goal_reached_ = false;
      publish_state("DRIVING");
    }
  }

  void publish_stop_and_state(const std::string & state)
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    publish_state(state);
  }

  void publish_state(const std::string & state)
  {
    if (state == last_state_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = state;
    state_pub_->publish(msg);
    last_state_ = state;
  }

  Limits limits_;
  double target_x_{0.0};
  double target_y_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double odom_timeout_{0.5};
  std::string goal_frame_{"odom"};
  bool odom_received_{false};
  bool goal_received_{false};
  bool goal_reached_{false};
  std::string last_state_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agv_task_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_task_controller::TaskController>());
  rclcpp::shutdown();
  return 0;
}
