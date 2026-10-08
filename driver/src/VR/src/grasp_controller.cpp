#include "grasp_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

using namespace std::chrono_literals;

GraspController::GraspController(rclcpp::Node &node) : node_(node) {
  open_position_ =
      node_.declare_parameter<double>("gripper_open_position", -4.6);
  closed_position_ =
      node_.declare_parameter<double>("gripper_closed_position", -0.0);
  torque_max_ =
      std::abs(node_.declare_parameter<double>("gripper_torque_max", 1.5));
  close_torque_ =
      std::abs(node_.declare_parameter<double>("gripper_close_torque", 1.0));
  hold_torque_ =
      std::abs(node_.declare_parameter<double>("gripper_hold_torque", 0.30));
  move_kp_ =
      std::max(0.0, node_.declare_parameter<double>("gripper_move_kp", 5.0));
  move_kd_ =
      std::max(0.0, node_.declare_parameter<double>("gripper_move_kd", 1.0));
  close_kp_ =
      std::max(0.0, node_.declare_parameter<double>("gripper_close_kp", 0.0));
  close_kd_ =
      std::max(0.0, node_.declare_parameter<double>("gripper_close_kd", 0.5));
  contact_torque_threshold_ = std::abs(
      node_.declare_parameter<double>("gripper_contact_torque_threshold", 0.45));
  close_threshold_ =
      node_.declare_parameter<double>("gripper_close_threshold", 0.7);
  open_threshold_ =
      node_.declare_parameter<double>("gripper_open_threshold", 0.3);
  arrive_tolerance_ = std::abs(
      node_.declare_parameter<double>("gripper_arrive_tolerance", 0.12));
  stall_velocity_ = std::abs(
      node_.declare_parameter<double>("gripper_stall_velocity", 0.05));
  startup_distance_ = std::abs(
      node_.declare_parameter<double>("gripper_startup_distance", 0.30));
  endpoint_tolerance_ = std::abs(
      node_.declare_parameter<double>("gripper_endpoint_tolerance", 0.05));

  if (open_position_ >= closed_position_) {
    throw std::invalid_argument(
        "gripper_open_position必须小于gripper_closed_position");
  }

  publisher_ =
      node_.create_publisher<rebotarm_interfaces::msg::JointMitCmd>(
          "/rebotarm/gripper/cmd/mit", 10);

  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
  command_subscription_ = node_.create_subscription<std_msgs::msg::Float32>(
      "/pico_right_controller/trigger", sensor_qos,
      std::bind(&GraspController::commandCallback, this,
                std::placeholders::_1));
  feedback_subscription_ = node_.create_subscription<
      rebotarm_interfaces::msg::JointMotorState>(
      "/rebotarm/gripper/state", sensor_qos,
      std::bind(&GraspController::feedbackCallback, this,
                std::placeholders::_1));

  control_timer_ = node_.create_wall_timer(
      20ms, std::bind(&GraspController::controlTick, this));

  RCLCPP_INFO(
      node_.get_logger(),
      "官方夹爪抓取状态机已启动：打开位置=%.3f，关闭位置=%.3f，"
      "闭合力矩=%.3f Nm，保持力矩=%.3f Nm",
      open_position_, closed_position_, close_torque_, hold_torque_);
}

void GraspController::commandCallback(
    const std_msgs::msg::Float32::SharedPtr message) {
  const double trigger = std::clamp<double>(message->data, 0.0, 1.0);
  Request next_request = Request::UNKNOWN;

  if (trigger >= close_threshold_) {
    next_request = Request::CLOSED;
  } else if (trigger <= open_threshold_) {
    next_request = Request::OPEN;
  } else {
    return;
  }

  if (next_request == request_ && !request_pending_) {
    return;
  }

  request_ = next_request;
  request_pending_ = true;

  // 与官方start()先等待反馈再开始控制的行为一致。这里先记录请求，收到
  // 第一帧真实反馈后再进入对应状态，绝不盲发绝对位置。
  if (!feedback_received_) {
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "已记录夹爪请求，等待第一帧真实反馈后执行");
    return;
  }

  if (request_ == Request::OPEN) {
    beginPosition(open_position_);
  } else {
    beginClosing();
  }
  request_pending_ = false;
}

void GraspController::feedbackCallback(
    const rebotarm_interfaces::msg::JointMotorState::SharedPtr message) {
  if (!std::isfinite(message->position) ||
      !std::isfinite(message->velocity) ||
      !std::isfinite(message->torque)) {
    return;
  }

  position_ = message->position;
  velocity_ = message->velocity;
  measured_torque_ = message->torque;
  feedback_received_ = true;

  if (request_pending_) {
    if (request_ == Request::OPEN) {
      beginPosition(open_position_);
    } else if (request_ == Request::CLOSED) {
      beginClosing();
    }
    request_pending_ = false;
  }
}

void GraspController::beginPosition(double target) {
  target_position_ = std::clamp(target, open_position_, closed_position_);
  state_ = State::POSITION;
  position_reached_ = false;
  RCLCPP_INFO(node_.get_logger(), "夹爪进入位置控制：目标=%.3f rad",
              target_position_);
}

void GraspController::beginClosing() {
  start_position_ = position_;
  contact_position_ = position_;
  target_position_ = closed_position_;

  // 官方视觉流程总是先打开再抓取；VR则可能在夹爪已经关闭时直接收到
  // 关闭请求。此时不应继续向机械端点施加接近力矩。
  if (position_ >= closed_position_ - endpoint_tolerance_) {
    state_ = State::POSITION;
    position_reached_ = true;
    RCLCPP_WARN(node_.get_logger(),
                "夹爪已经位于关闭端，不再施加闭合力矩：位置=%.3f rad",
                position_);
    return;
  }

  state_ = State::CLOSING;
  position_reached_ = false;
  RCLCPP_INFO(
      node_.get_logger(),
      "夹爪开始恒力矩闭合：起点=%.3f rad，tau=%.3f Nm",
      start_position_, close_torque_);
}

void GraspController::controlTick() {
  if (!feedback_received_) {
    return;
  }

  switch (state_) {
    case State::IDLE:
      return;

    case State::POSITION: {
      publishMit(target_position_, 0.0, move_kp_, move_kd_, 0.0);
      if (!position_reached_ &&
          std::abs(position_ - target_position_) < arrive_tolerance_) {
        position_reached_ = true;
        RCLCPP_INFO(node_.get_logger(),
                    "夹爪到达位置目标：位置=%.3f rad", position_);
      }
      return;
    }

    case State::CLOSING: {
      // 默认close_kp=0，与官方恒力矩闭合方案一致；也可通过参数调整。
      publishMit(closed_position_, 0.0, close_kp_, close_kd_, close_torque_);
      contact_position_ = position_;

      const bool moved =
          std::abs(position_ - start_position_) >= startup_distance_;
      const bool at_closed_endpoint =
          position_ >= closed_position_ - endpoint_tolerance_;

      if (moved && at_closed_endpoint) {
        // 到达关闭端说明没有物体阻挡；撤销恒定力矩，改为位置保持。
        beginPosition(closed_position_);
        RCLCPP_WARN(node_.get_logger(),
                    "夹爪到达关闭端，未检测到物体：位置=%.3f rad",
                    position_);
      } else if (
          moved &&
          (std::abs(velocity_) < stall_velocity_ ||
           measured_torque_ > contact_torque_threshold_)) {
        // 在合法行程内部速度足够低，或闭合方向反馈力矩超过阈值，
        // 均认为已经接触物体；保存接触位置并降低保持力矩。
        target_position_ = std::clamp(
            contact_position_, open_position_, closed_position_);
        state_ = State::HOLDING;
        RCLCPP_INFO(
            node_.get_logger(),
            "夹爪检测到物体并进入保持：位置=%.3f rad，反馈力矩=%.3f Nm，"
            "保持力矩=%.3f Nm",
            position_, measured_torque_, hold_torque_);
      }
      return;
    }

    case State::HOLDING:
      publishMit(target_position_, 0.0, move_kp_, move_kd_, hold_torque_);
      return;
  }
}

void GraspController::publishMit(double position, double velocity, double kp,
                                 double kd, double torque) {
  rebotarm_interfaces::msg::JointMitCmd command;
  command.pos = std::clamp(position, open_position_, closed_position_);
  command.vel = velocity;
  command.kp = std::max(0.0, kp);
  command.kd = std::max(0.0, kd);
  command.tau = std::clamp(torque, -torque_max_, torque_max_);
  command.stamp = node_.now();
  publisher_->publish(command);
}
