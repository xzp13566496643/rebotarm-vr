#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rebotarm_msgs/msg/joint_pos_vel_cmd.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

class VrToServo : public rclcpp::Node
{
public:
  VrToServo() : Node("vr_to_servo")
  {
    position_scale_ = declare_parameter("position_scale", 0.80);
    max_linear_speed_ = declare_parameter("max_linear_speed", 0.10);
    orientation_scale_ = declare_parameter("orientation_scale", 1.0);
    max_angular_speed_ = declare_parameter("max_angular_speed", 0.60);
    grip_threshold_ = declare_parameter("grip_threshold", 0.50);
    input_timeout_ = declare_parameter("input_timeout", 0.20);
    filter_alpha_ = declare_parameter("filter_alpha", 0.25);
    gripper_open_ = declare_parameter("gripper_open", 0.045);
    gripper_closed_ = declare_parameter("gripper_closed", 0.0);
    hardware_gripper_ = declare_parameter("hardware_gripper", false);
    gripper_motor_open_ = declare_parameter("gripper_motor_open", -5.0);
    gripper_motor_closed_ = declare_parameter("gripper_motor_closed", 0.0);
    gripper_motor_vlim_ = declare_parameter("gripper_motor_vlim", 2.0);

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);
    gripper_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/gripper_controller/joint_trajectory", 10);
    hardware_gripper_pub_ = create_publisher<rebotarm_msgs::msg::JointPosVelCmd>(
      "/rebotarm/gripper/cmd/pos_vel", 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pico_right_controller/pose", rclcpp::SensorDataQoS(),
      std::bind(&VrToServo::poseCallback, this, std::placeholders::_1));
    grip_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/pico_left_controller/trigger", rclcpp::SensorDataQoS(),
      std::bind(&VrToServo::gripCallback, this, std::placeholders::_1));
    trigger_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/pico_right_controller/trigger", rclcpp::SensorDataQoS(),
      std::bind(&VrToServo::triggerCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(20ms, std::bind(&VrToServo::publishCommand, this));
    RCLCPP_INFO(
      get_logger(),
      "VR simulation mapping ready: hold left trigger to move with right controller; "
      "right trigger controls gripper");
  }

private:
  static double clamp(double value, double limit)
  {
    if (limit <= 0.0) {
      return value;
    }
    return std::clamp(value, -limit, limit);
  }

  void gripCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const bool next_engaged = msg->data >= grip_threshold_;
    if (next_engaged && !engaged_) {
      have_previous_pose_ = false;
      filtered_velocity_.fill(0.0);
      filtered_angular_velocity_.fill(0.0);
      RCLCPP_INFO(get_logger(), "VR clutch engaged");
    } else if (!next_engaged && engaged_) {
      filtered_velocity_.fill(0.0);
      filtered_angular_velocity_.fill(0.0);
      RCLCPP_INFO(get_logger(), "VR clutch released");
    }
    engaged_ = next_engaged;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto receipt_time = now();
    last_input_time_ = receipt_time;

    if (!engaged_ || !have_previous_pose_) {
      previous_position_ = {
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
      previous_orientation_ = {
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w};
      previous_pose_time_ = receipt_time;
      have_previous_pose_ = true;
      return;
    }

    const double dt = (receipt_time - previous_pose_time_).seconds();
    if (dt <= 1e-4 || dt > input_timeout_) {
      previous_position_ = {
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
      previous_orientation_ = {
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w};
      previous_pose_time_ = receipt_time;
      filtered_velocity_.fill(0.0);
      filtered_angular_velocity_.fill(0.0);
      return;
    }

    const double vr_dx = msg->pose.position.x - previous_position_[0];
    const double vr_dy = msg->pose.position.y - previous_position_[1];
    const double vr_dz = msg->pose.position.z - previous_position_[2];

    // OpenXR: +X right, +Y up, -Z forward.
    // reBot base: +X forward, +Y left, +Z up.
    const std::array<double, 3> raw_velocity{
      position_scale_ * (-vr_dz) / dt,
      position_scale_ * (-vr_dx) / dt,
      position_scale_ * vr_dy / dt};

    for (std::size_t i = 0; i < filtered_velocity_.size(); ++i) {
      const double limited = clamp(raw_velocity[i], max_linear_speed_);
      filtered_velocity_[i] =
        filter_alpha_ * limited + (1.0 - filter_alpha_) * filtered_velocity_[i];
    }

    // Relative controller rotation in the OpenXR world frame:
    // q_delta = q_current * inverse(q_previous).  Quaternion storage is xyzw.
    const std::array<double, 4> current_orientation{
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w};
    const auto & p = previous_orientation_;
    const auto & c = current_orientation;
    double qx = -c[3] * p[0] + c[0] * p[3] - c[1] * p[2] + c[2] * p[1];
    double qy = -c[3] * p[1] + c[0] * p[2] + c[1] * p[3] - c[2] * p[0];
    double qz = -c[3] * p[2] - c[0] * p[1] + c[1] * p[0] + c[2] * p[3];
    double qw =  c[3] * p[3] + c[0] * p[0] + c[1] * p[1] + c[2] * p[2];
    const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    std::array<double, 3> vr_angular_velocity{};
    if (qnorm > 1e-9) {
      qx /= qnorm;
      qy /= qnorm;
      qz /= qnorm;
      qw /= qnorm;
      if (qw < 0.0) {
        qx = -qx;
        qy = -qy;
        qz = -qz;
        qw = -qw;
      }
      const double vector_norm = std::sqrt(qx * qx + qy * qy + qz * qz);
      if (vector_norm > 1e-9) {
        const double angle = 2.0 * std::atan2(vector_norm, std::clamp(qw, -1.0, 1.0));
        const double gain = orientation_scale_ * angle / (vector_norm * dt);
        vr_angular_velocity = {gain * qx, gain * qy, gain * qz};
      }
    }

    // Apply the same OpenXR-to-reBot basis mapping used for translation.
    const std::array<double, 3> raw_angular_velocity{
      -vr_angular_velocity[2],
      -vr_angular_velocity[0],
      vr_angular_velocity[1]};
    for (std::size_t i = 0; i < filtered_angular_velocity_.size(); ++i) {
      const double limited = clamp(raw_angular_velocity[i], max_angular_speed_);
      filtered_angular_velocity_[i] =
        filter_alpha_ * limited +
        (1.0 - filter_alpha_) * filtered_angular_velocity_[i];
    }

    previous_position_ = {
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    previous_orientation_ = current_orientation;
    previous_pose_time_ = receipt_time;
  }

  void triggerCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const double trigger = std::clamp(static_cast<double>(msg->data), 0.0, 1.0);
    if (hardware_gripper_) {
      double target = 0.0;
      if (trigger >= 0.70) {
        target = gripper_motor_closed_;
      } else if (trigger <= 0.30) {
        target = gripper_motor_open_;
      } else {
        return;
      }
      if (std::abs(target - last_gripper_target_) < 0.02) {
        return;
      }
      rebotarm_msgs::msg::JointPosVelCmd command;
      command.pos = target;
      command.vlim = gripper_motor_vlim_;
      hardware_gripper_pub_->publish(command);
      last_gripper_target_ = target;
      return;
    }
    const double target = gripper_open_ + trigger * (gripper_closed_ - gripper_open_);
    if (std::abs(target - last_gripper_target_) < 0.001) {
      return;
    }

    trajectory_msgs::msg::JointTrajectory command;
    command.joint_names = {"gripper_joint1", "gripper_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {target, target};
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 100000000;
    command.points.push_back(point);
    gripper_pub_->publish(command);
    last_gripper_target_ = target;
  }

  void publishCommand()
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";

    const bool fresh =
      last_input_time_.nanoseconds() > 0 &&
      (now() - last_input_time_).seconds() <= input_timeout_;
    if (engaged_ && fresh) {
      command.twist.linear.x = filtered_velocity_[0];
      command.twist.linear.y = filtered_velocity_[1];
      command.twist.linear.z = filtered_velocity_[2];
      command.twist.angular.x = filtered_angular_velocity_[0];
      command.twist.angular.y = filtered_angular_velocity_[1];
      command.twist.angular.z = filtered_angular_velocity_[2];
    } else {
      filtered_velocity_.fill(0.0);
      filtered_angular_velocity_.fill(0.0);
    }
    twist_pub_->publish(command);
  }

  double position_scale_;
  double max_linear_speed_;
  double orientation_scale_;
  double max_angular_speed_;
  double grip_threshold_;
  double input_timeout_;
  double filter_alpha_;
  double gripper_open_;
  double gripper_closed_;
  bool hardware_gripper_;
  double gripper_motor_open_;
  double gripper_motor_closed_;
  double gripper_motor_vlim_;
  double last_gripper_target_{-1.0};
  bool engaged_{false};
  bool have_previous_pose_{false};
  std::array<double, 3> previous_position_{};
  std::array<double, 3> filtered_velocity_{};
  std::array<double, 3> filtered_angular_velocity_{};
  std::array<double, 4> previous_orientation_{0.0, 0.0, 0.0, 1.0};
  rclcpp::Time previous_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  rclcpp::Publisher<rebotarm_msgs::msg::JointPosVelCmd>::SharedPtr hardware_gripper_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr grip_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VrToServo>());
  rclcpp::shutdown();
  return 0;
}
