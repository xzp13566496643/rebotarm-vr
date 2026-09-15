#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rebotarm_interfaces/msg/joint_pos_vel_cmd.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace {

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Vector3 mapVrVector(const Vector3 &value) {
  // OpenXR：+X 向右、+Y 向上、-Z 向前。
  // reBot base_link：+X 向前、+Y 向左、+Z 向上。
  return {-value.z, -value.x, value.y};
}

Vector3 clampNorm(const Vector3 &value, double limit) {
  const double norm = std::sqrt(
      value.x * value.x + value.y * value.y + value.z * value.z);
  if (limit > 0.0 && norm > limit) {
    const double scale = limit / norm;
    return {value.x * scale, value.y * scale, value.z * scale};
  }
  return value;
}

tf2::Quaternion normalized(tf2::Quaternion value) {
  if (value.length2() < 1.0e-12) {
    return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
  }
  value.normalize();
  return value;
}

tf2::Quaternion poseQuaternion(const geometry_msgs::msg::Pose &pose) {
  tf2::Quaternion result;
  tf2::fromMsg(pose.orientation, result);
  return normalized(result);
}

}  // namespace

class VrToControlNode : public rclcpp::Node {
 public:
  VrToControlNode()
      : Node("vr_to_control"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    // VR累计目标和末端误差控制参数，与旧Python真机版本保持一致。
    position_scale_ = declare_parameter<double>("position_scale", 0.8);
    orientation_scale_ = declare_parameter<double>("orientation_scale", 1.0);
    position_gain_ = declare_parameter<double>("position_gain", 3.0);
    orientation_gain_ = declare_parameter<double>("orientation_gain", 3.0);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.4);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.8);
    input_timeout_ = declare_parameter<double>("input_timeout", 0.2);
    clutch_threshold_ = declare_parameter<double>("clutch_threshold", 0.5);

    // 夹爪只做基础POS_VEL开合；反馈和抓取判断不属于本节点。
    gripper_open_position_ =
        declare_parameter<double>("gripper_open_position", -4.6);
    gripper_closed_position_ =
        declare_parameter<double>("gripper_closed_position", 0.0);
    gripper_velocity_limit_ =
        declare_parameter<double>("gripper_velocity_limit", 2.0);
    gripper_close_threshold_ =
        declare_parameter<double>("gripper_close_threshold", 0.7);
    gripper_open_threshold_ =
        declare_parameter<double>("gripper_open_threshold", 0.3);

    twist_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
    gripper_publisher_ =
        create_publisher<rebotarm_interfaces::msg::JointPosVelCmd>(
            "/rebotarm/gripper/cmd/pos_vel", 10);

    // VR属于实时状态流，只保留最新一帧，避免旧手柄数据积压。
    auto vr_qos = rclcpp::SensorDataQoS().keep_last(1);
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pico_right_controller/pose", vr_qos,
        std::bind(&VrToControlNode::poseCallback, this,
                  std::placeholders::_1));
    clutch_subscription_ = create_subscription<std_msgs::msg::Float32>(
        "/pico_left_controller/trigger", vr_qos,
        std::bind(&VrToControlNode::clutchCallback, this,
                  std::placeholders::_1));
    gripper_subscription_ = create_subscription<std_msgs::msg::Float32>(
        "/pico_right_controller/trigger", vr_qos,
        std::bind(&VrToControlNode::gripperCallback, this,
                  std::placeholders::_1));

    control_timer_ = create_wall_timer(
        20ms, std::bind(&VrToControlNode::controlLoop, this));

    RCLCPP_INFO(
        get_logger(),
        "VR累计位姿控制已启动：左扳机控制机械臂，右扳机控制夹爪");
  }

 private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message) {
    // 缓存右手柄位姿；离合接合期间用它更新持续TCP目标。
    latest_hand_pose_ = message->pose;
    hand_pose_received_ = true;
    last_hand_time_ = now();

    if (engaged_ && reference_valid_) {
      updateTarget();
    }
  }

  bool lookupTcp(geometry_msgs::msg::Pose &pose) {
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
          get_logger(), *get_clock(), 2000,
          "无法查询base_link到gripper_tcp的TF：%s", error.what());
      return false;
    }
  }

  void clutchCallback(const std_msgs::msg::Float32::SharedPtr message) {
    // 处理左扳机离合；接合瞬间记录手柄和TCP的两组起始基准。
    const bool requested = message->data >= clutch_threshold_;

    if (requested && !engaged_) {
      if (!hand_pose_received_) {
        RCLCPP_WARN(get_logger(), "无法接合VR控制：尚未收到右手柄位姿");
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
      RCLCPP_INFO(get_logger(), "VR离合已接合并记录起始基准");
      return;
    }

    if (!requested && engaged_) {
      engaged_ = false;
      reference_valid_ = false;
      publishZeroTwist();
      RCLCPP_INFO(get_logger(), "VR离合已释放");
    }
  }

  void updateTarget() {
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
    const tf2::Quaternion delta_vr = normalized(hand_now * hand_start.inverse());

    // 与旧Python实现相同：对相对四元数向量部做轴置换和符号变换。
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

  void controlLoop() {
    // 以50Hz根据TCP剩余位姿误差生成受限Twist并交给MoveIt Servo。
    if (!engaged_ || !reference_valid_ || !hand_pose_received_) {
      publishZeroTwist();
      return;
    }

    if ((now() - last_hand_time_).seconds() > input_timeout_) {
      publishZeroTwist();
      return;
    }

    geometry_msgs::msg::Pose current_pose;
    if (!lookupTcp(current_pose)) {
      publishZeroTwist();
      return;
    }

    Vector3 linear{
        position_gain_ * (target_pose_.position.x - current_pose.position.x),
        position_gain_ * (target_pose_.position.y - current_pose.position.y),
        position_gain_ * (target_pose_.position.z - current_pose.position.z)};
    linear = clampNorm(linear, max_linear_speed_);

    const tf2::Quaternion target = poseQuaternion(target_pose_);
    const tf2::Quaternion current = poseQuaternion(current_pose);
    tf2::Quaternion error = normalized(target * current.inverse());
    if (error.w() < 0.0) {
      error = tf2::Quaternion(-error.x(), -error.y(), -error.z(), -error.w());
    }

    const double vector_norm = std::sqrt(
        error.x() * error.x() + error.y() * error.y() +
        error.z() * error.z());
    Vector3 angular{};
    if (vector_norm > 1.0e-9) {
      const double angle =
          2.0 * std::atan2(vector_norm, std::clamp(error.w(), -1.0, 1.0));
      angular = {
          orientation_gain_ * angle * error.x() / vector_norm,
          orientation_gain_ * angle * error.y() / vector_norm,
          orientation_gain_ * angle * error.z() / vector_norm};
      angular = clampNorm(angular, max_angular_speed_);
    }

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    command.twist.linear.x = linear.x;
    command.twist.linear.y = linear.y;
    command.twist.linear.z = linear.z;
    command.twist.angular.x = angular.x;
    command.twist.angular.y = angular.y;
    command.twist.angular.z = angular.z;
    twist_publisher_->publish(command);
  }

  void publishZeroTwist() {
    // 发布零速度，使MoveIt Servo停止继续运动。
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    twist_publisher_->publish(command);
  }

  void gripperCallback(const std_msgs::msg::Float32::SharedPtr message) {
    // 把右扳机转换为带回差的夹爪POS_VEL开合命令。
    const double trigger = std::clamp<double>(message->data, 0.0, 1.0);
    double target = 0.0;
    GripperRequest request = GripperRequest::UNKNOWN;

    if (trigger >= gripper_close_threshold_) {
      target = gripper_closed_position_;
      request = GripperRequest::CLOSED;
    } else if (trigger <= gripper_open_threshold_) {
      target = gripper_open_position_;
      request = GripperRequest::OPEN;
    } else {
      return;
    }

    if (request == last_gripper_request_) {
      return;
    }

    rebotarm_interfaces::msg::JointPosVelCmd command;
    command.pos = target;
    command.vlim = std::max(0.0, gripper_velocity_limit_);
    command.stamp = now();
    gripper_publisher_->publish(command);
    last_gripper_request_ = request;

    RCLCPP_INFO(
        get_logger(), "夹爪目标：%s，位置=%.3f rad，速度上限=%.3f rad/s",
        request == GripperRequest::CLOSED ? "关闭" : "打开", target,
        command.vlim);
  }

  enum class GripperRequest { UNKNOWN, OPEN, CLOSED };

  double position_scale_{0.8};
  double orientation_scale_{1.0};
  double position_gain_{3.0};
  double orientation_gain_{3.0};
  double max_linear_speed_{0.4};
  double max_angular_speed_{0.8};
  double input_timeout_{0.2};
  double clutch_threshold_{0.5};
  double gripper_open_position_{-4.6};
  double gripper_closed_position_{0.0};
  double gripper_velocity_limit_{2.0};
  double gripper_close_threshold_{0.7};
  double gripper_open_threshold_{0.3};

  bool hand_pose_received_{false};
  bool engaged_{false};
  bool reference_valid_{false};
  geometry_msgs::msg::Pose latest_hand_pose_;
  geometry_msgs::msg::Pose hand_start_pose_;
  geometry_msgs::msg::Pose robot_start_pose_;
  geometry_msgs::msg::Pose target_pose_;
  rclcpp::Time last_hand_time_{0, 0, RCL_ROS_TIME};
  GripperRequest last_gripper_request_{GripperRequest::UNKNOWN};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      twist_publisher_;
  rclcpp::Publisher<rebotarm_interfaces::msg::JointPosVelCmd>::SharedPtr
      gripper_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      clutch_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      gripper_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VrToControlNode>());
  rclcpp::shutdown();
  return 0;
}
