#pragma once

#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rebotarm_interfaces/msg/joint_mit_cmd.hpp"
#include "rebotarm_interfaces/msg/joint_motor_state.hpp"
#include "std_msgs/msg/float32.hpp"

/**
 * 按Seeed官方GraspDriver状态机控制夹爪。
 *
 * 打开时使用MIT位置控制；关闭时先施加闭合力矩，检测到行程内部堵转后，
 * 保存接触位置并切换为较小保持力矩。该类只收发ROS话题，不创建硬件连接。
 */
class GraspController {
 public:
  explicit GraspController(rclcpp::Node &node);

 private:
  enum class State { IDLE, POSITION, CLOSING, HOLDING };
  enum class Request { UNKNOWN, OPEN, CLOSED };

  /** 将右扳机转换成打开或抓取状态请求。 */
  void commandCallback(const std_msgs::msg::Float32::SharedPtr message);

  /** 缓存夹爪电机的位置、速度和力矩反馈。 */
  void feedbackCallback(
      const rebotarm_interfaces::msg::JointMotorState::SharedPtr message);

  /** 每20 ms依据当前状态生成一帧MIT命令。 */
  void controlTick();

  /** 发布经过位置软限位和力矩上限裁剪的MIT命令。 */
  void publishMit(double position, double velocity, double kp, double kd,
                  double torque);

  /** 进入位置控制，用于打开夹爪或在关闭端保持。 */
  void beginPosition(double target);

  /** 进入恒力矩闭合阶段，并记录本轮闭合起点。 */
  void beginClosing();

  rclcpp::Node &node_;

  double open_position_{-4.6};
  double closed_position_{-0.1};
  double torque_max_{1.5};
  double close_torque_{1.0};
  double hold_torque_{0.30};
  double move_kp_{5.0};
  double move_kd_{1.0};
  double close_kp_{0.0};
  double close_kd_{0.5};
  double contact_torque_threshold_{0.45};
  double close_threshold_{0.7};
  double open_threshold_{0.3};
  double arrive_tolerance_{0.12};
  double stall_velocity_{0.05};
  double startup_distance_{0.30};
  double endpoint_tolerance_{0.05};

  State state_{State::IDLE};
  Request request_{Request::UNKNOWN};
  bool feedback_received_{false};
  bool request_pending_{false};
  bool position_reached_{true};
  double position_{0.0};
  double velocity_{0.0};
  double measured_torque_{0.0};
  double target_position_{0.0};
  double start_position_{0.0};
  double contact_position_{0.0};

  rclcpp::Publisher<rebotarm_interfaces::msg::JointMitCmd>::SharedPtr
      publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      command_subscription_;
  rclcpp::Subscription<rebotarm_interfaces::msg::JointMotorState>::SharedPtr
      feedback_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};
