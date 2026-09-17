#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/** 使用VR累计相对位姿生成MoveIt Servo末端速度命令。 */
class ArmController {
 public:
  /** 创建机械臂VR累计位姿和MoveIt Servo速度控制。 */
  explicit ArmController(rclcpp::Node &node);

 private:
  /** 缓存右手柄的最新绝对位姿，并更新累计TCP目标。 */
  void poseCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr message);

  /** 处理左扳机离合，并在接合瞬间记录手柄和TCP基准。 */
  void clutchCallback(const std_msgs::msg::Float32::SharedPtr message);

  /** 从TF查询base_link坐标系中的当前gripper_tcp位姿。 */
  bool lookupTcp(geometry_msgs::msg::Pose &pose);

  /** 将手柄相对起点的累计变化映射为持久TCP目标。 */
  void updateTarget();

  /** 以50 Hz将TCP剩余误差转换成受限Twist并发布给Servo。 */
  void controlLoop();

  /** 发布零Twist，使MoveIt Servo停止继续运动。 */
  void publishZeroTwist();

  rclcpp::Node &node_;
  double position_scale_{0.8};
  double orientation_scale_{1.0};
  double position_gain_{3.0};
  double orientation_gain_{3.0};
  double max_linear_speed_{0.4};
  double max_angular_speed_{0.8};
  double input_timeout_{0.2};
  double clutch_threshold_{0.5};

  bool hand_pose_received_{false};
  bool engaged_{false};
  bool reference_valid_{false};
  geometry_msgs::msg::Pose latest_hand_pose_;
  geometry_msgs::msg::Pose hand_start_pose_;
  geometry_msgs::msg::Pose robot_start_pose_;
  geometry_msgs::msg::Pose target_pose_;
  rclcpp::Time last_hand_time_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      twist_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      clutch_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};
