#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.arguments(
    {"--ros-args", "-r", "joint_states:=/rebotarm/joint_states"});
  auto node = std::make_shared<rclcpp::Node>("move_tcp_up", node_options);

  const double distance = node->declare_parameter<double>("distance", 0.10);
  const bool execute = node->declare_parameter<bool>("execute", false);
  const double velocity_scaling =
    node->declare_parameter<double>("velocity_scaling", 0.05);
  const double acceleration_scaling =
    node->declare_parameter<double>("acceleration_scaling", 0.05);

  // MoveGroupInterface waits for joint-state, TF, service, and action callbacks.
  // Spin this node in a separate thread while plan()/execute() are blocking.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  int exit_code = 1;
  try {
    MoveGroupInterface arm(node, "arm");
    arm.setEndEffectorLink("gripper_tcp");
    arm.setPoseReferenceFrame("base_link");
    arm.setPlanningPipelineId("ompl");
    arm.setPlannerId("RRTConnectkConfigDefault");
    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(20);
    arm.setMaxVelocityScalingFactor(velocity_scaling);
    arm.setMaxAccelerationScalingFactor(acceleration_scaling);
    arm.setStartStateToCurrentState();

    const auto current = arm.getCurrentPose("gripper_tcp");
    const double target_x = -0.021293;
    const double target_y = -0.14425;
    const double target_z = 0.75253;

    RCLCPP_INFO(
      node->get_logger(),
      "Current TCP: x=%.4f y=%.4f z=%.4f; target: x=%.4f y=%.4f z=%.4f",
      current.pose.position.x,
      current.pose.position.y,
      current.pose.position.z,
      target_x,
      target_y,
      target_z);

    // Position-only target: MoveIt is free to choose the TCP orientation.
    if (!arm.setPositionTarget(target_x, target_y, target_z, "gripper_tcp")) {
      RCLCPP_ERROR(node->get_logger(), "MoveIt rejected the position target");
      throw std::runtime_error("setPositionTarget failed");
    }

    MoveGroupInterface::Plan plan;
    const auto plan_result = arm.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        node->get_logger(), "MoveIt planning failed, error code: %d",
        plan_result.val);
      throw std::runtime_error("planning failed");
    }

    RCLCPP_INFO(
      node->get_logger(), "Planning succeeded with %zu trajectory points",
      plan.trajectory_.joint_trajectory.points.size());

    if (!execute) {
      RCLCPP_WARN(
        node->get_logger(),
        "Plan-only mode: robot will not move. Set execute:=true to execute.");
      exit_code = 0;
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "Executing real-arm trajectory: target TCP Z increases by %.3f m",
        distance);
      const auto execute_result = arm.execute(plan);
      if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          node->get_logger(), "Trajectory execution failed, error code: %d",
          execute_result.val);
        throw std::runtime_error("execution failed");
      }
      RCLCPP_INFO(node->get_logger(), "Trajectory execution succeeded");
      exit_code = 0;
    }

    arm.clearPoseTargets();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
