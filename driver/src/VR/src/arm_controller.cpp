#include "arm_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace {

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/** 将OpenXR坐标轴映射到reBot的base_link坐标轴。 */
Vector3 mapVrVector(const Vector3 &value) {
  // OpenXR：+X向右、+Y向上、-Z向前。
  // reBot base_link：+X向前、+Y向左、+Z向上。
  return {-value.z, -value.x, value.y};
}

/** 在保持向量方向不变的前提下限制其模长。 */
Vector3 clampNorm(const Vector3 &value, double limit) {
  const double norm = std::sqrt(
      value.x * value.x + value.y * value.y + value.z * value.z);
  if (limit > 0.0 && norm > limit) {
    const double scale = limit / norm;
    return {value.x * scale, value.y * scale, value.z * scale};
  }
  return value;
}

/** 返回单位四元数；无效的零四元数回退为单位旋转。 */
tf2::Quaternion normalized(tf2::Quaternion value) {
  if (value.length2() < 1.0e-12) {
    return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  }
  value.normalize();
  return value;
}

/** 从ROS Pose取出并归一化姿态四元数。 */
tf2::Quaternion poseQuaternion(const geometry_msgs::msg::Pose &pose) {
  tf2::Quaternion result;
  tf2::fromMsg(pose.orientation, result);
  return normalized(result);
}

}  // namespace

ArmController::ArmController(rclcpp::Node &node)
    : node_(node),
      tf_buffer_(node.get_clock()),
      tf_listener_(tf_buffer_) {
  // VR累计目标和末端误差控制参数，与旧Python真机版本保持一致。
  position_scale_ = node_.declare_parameter<double>("position_scale", 0.8);
  orientation_scale_ =
      node_.declare_parameter<double>("orientation_scale", 1.0);
  position_gain_ = node_.declare_parameter<double>("position_gain", 3.0);
  orientation_gain_ =
      node_.declare_parameter<double>("orientation_gain", 3.0);
  max_linear_speed_ =
      node_.declare_parameter<double>("max_linear_speed", 0.4);
  max_angular_speed_ =
      node_.declare_parameter<double>("max_angular_speed", 0.8);
  input_timeout_ = node_.declare_parameter<double>("input_timeout", 0.2);
  clutch_threshold_ =
      node_.declare_parameter<double>("clutch_threshold", 0.5);

  twist_publisher_ =
      node_.create_publisher<geometry_msgs::msg::TwistStamped>(
          "/servo_node/delta_twist_cmds", 10);

  // VR属于实时状态流，只保留最新一帧，避免旧手柄数据积压。
  auto vr_qos = rclcpp::SensorDataQoS().keep_last(1);
  pose_subscription_ =
      node_.create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pico_right_controller/pose", vr_qos,
          std::bind(&ArmController::poseCallback, this,
                    std::placeholders::_1));
  clutch_subscription_ = node_.create_subscription<std_msgs::msg::Float32>(
      "/pico_left_controller/trigger", vr_qos,
      std::bind(&ArmController::clutchCallback, this,
                std::placeholders::_1));
  control_timer_ = node_.create_wall_timer(
      20ms, std::bind(&ArmController::controlLoop, this));

  RCLCPP_INFO(node_.get_logger(), "机械臂VR累计位姿控制已启动");
}

void ArmController::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr message) {
  // 缓存右手柄位姿；离合接合期间用它更新持续TCP目标。
  latest_hand_pose_ = message->pose;
  hand_pose_received_ = true;
  last_hand_time_ = node_.now();

  if (engaged_ && reference_valid_) {
    updateTarget();
  }
}

bool ArmController::lookupTcp(geometry_msgs::msg::Pose &pose) {
  // 查询base_link到gripper_tcp的当前机械臂末端位姿。
  try {
    const auto transform = tf_buffer_.lookupTransform(
        "base_link", "gripper_tcp", tf2::TimePointZero);
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000,
        "无法查询base_link到gripper_tcp的TF：%s", error.what());
    return false;
  }
}

void ArmController::clutchCallback(
    const std_msgs::msg::Float32::SharedPtr message) {
  // 处理左扳机离合；接合瞬间记录手柄和TCP的两组起始基准。
  const bool requested = message->data >= clutch_threshold_;

  if (requested && !engaged_) {
    if (!hand_pose_received_) {
      RCLCPP_WARN(node_.get_logger(),
                  "无法接合VR控制：尚未收到右手柄位姿");
      return;
    }

    geometry_msgs::msg::Pose tcp_pose;
    if (!lookupTcp(tcp_pose)) {
      return;
    }

    hand_start_pose_ = latest_hand_pose_;
    robot_start_pose_ = tcp_pose;
    target_pose_ = tcp_pose;
    reference_valid_ = true;
    engaged_ = true;
    RCLCPP_INFO(node_.get_logger(), "VR离合已接合并记录起始基准");
    return;
  }

  if (!requested && engaged_) {
    engaged_ = false;
    reference_valid_ = false;
    publishZeroTwist();
    RCLCPP_INFO(node_.get_logger(), "VR离合已释放");
  }
}

void ArmController::updateTarget() {
  // 把手柄相对按下位置的累计变化映射成持久TCP目标位姿。
  const Vector3 hand_delta{
      latest_hand_pose_.position.x - hand_start_pose_.position.x,
      latest_hand_pose_.position.y - hand_start_pose_.position.y,
      latest_hand_pose_.position.z - hand_start_pose_.position.z};
  const Vector3 mapped_delta = mapVrVector(hand_delta);

  target_pose_.position.x =
      robot_start_pose_.position.x + position_scale_ * mapped_delta.x;
  target_pose_.position.y =
      robot_start_pose_.position.y + position_scale_ * mapped_delta.y;
  target_pose_.position.z =
      robot_start_pose_.position.z + position_scale_ * mapped_delta.z;

  const tf2::Quaternion hand_now = poseQuaternion(latest_hand_pose_);
  const tf2::Quaternion hand_start = poseQuaternion(hand_start_pose_);
  const tf2::Quaternion delta_vr =
      normalized(hand_now * hand_start.inverse());

  // 对手柄相对旋转的四元数向量部执行同一套坐标轴映射。
  const Vector3 mapped_rotation =
      mapVrVector({delta_vr.x(), delta_vr.y(), delta_vr.z()});
  tf2::Quaternion delta_robot(
      mapped_rotation.x * orientation_scale_,
      mapped_rotation.y * orientation_scale_,
      mapped_rotation.z * orientation_scale_, delta_vr.w());
  delta_robot = normalized(delta_robot);

  const tf2::Quaternion robot_start = poseQuaternion(robot_start_pose_);
  target_pose_.orientation =
      tf2::toMsg(normalized(delta_robot * robot_start));
}

void ArmController::controlLoop() {
  // 以50 Hz根据TCP剩余位姿误差生成受限Twist并交给MoveIt Servo。
  if (!engaged_ || !reference_valid_ || !hand_pose_received_) {
    publishZeroTwist();
    return;
  }

  if ((node_.now() - last_hand_time_).seconds() > input_timeout_) {
    publishZeroTwist();
    return;
  }

  geometry_msgs::msg::Pose current_pose;
  if (!lookupTcp(current_pose)) {
    publishZeroTwist();
    return;
  }

  Vector3 linear{
      position_gain_ *
          (target_pose_.position.x - current_pose.position.x),
      position_gain_ *
          (target_pose_.position.y - current_pose.position.y),
      position_gain_ *
          (target_pose_.position.z - current_pose.position.z)};
  linear = clampNorm(linear, max_linear_speed_);

  const tf2::Quaternion target = poseQuaternion(target_pose_);
  const tf2::Quaternion current = poseQuaternion(current_pose);
  tf2::Quaternion error = normalized(target * current.inverse());
  if (error.w() < 0.0) {
    error =
        tf2::Quaternion(-error.x(), -error.y(), -error.z(), -error.w());
  }

  const double vector_norm = std::sqrt(
      error.x() * error.x() + error.y() * error.y() +
      error.z() * error.z());
  Vector3 angular{};
  if (vector_norm > 1.0e-9) {
    const double angle =
        2.0 * std::atan2(vector_norm,
                         std::clamp(error.w(), -1.0, 1.0));
    angular = {
        orientation_gain_ * angle * error.x() / vector_norm,
        orientation_gain_ * angle * error.y() / vector_norm,
        orientation_gain_ * angle * error.z() / vector_norm};
    angular = clampNorm(angular, max_angular_speed_);
  }

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_.now();
  command.header.frame_id = "base_link";
  command.twist.linear.x = linear.x;
  command.twist.linear.y = linear.y;
  command.twist.linear.z = linear.z;
  command.twist.angular.x = angular.x;
  command.twist.angular.y = angular.y;
  command.twist.angular.z = angular.z;
  twist_publisher_->publish(command);
}

void ArmController::publishZeroTwist() {
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = node_.now();
  command.header.frame_id = "base_link";
  twist_publisher_->publish(command);
}
