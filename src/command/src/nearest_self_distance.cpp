#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nearest_self_distance");

  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  const auto robot_model = loader.getModel();
  if (!robot_model) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load robot model");
    rclcpp::shutdown();
    return 1;
  }

  sensor_msgs::msg::JointState joint_state;
  if (!rclcpp::wait_for_message(joint_state, node, "/joint_states", 5s)) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for /joint_states");
    rclcpp::shutdown();
    return 2;
  }

  planning_scene::PlanningScene scene(robot_model);
  auto & state = scene.getCurrentStateNonConst();
  state.setVariablePositions(joint_state.name, joint_state.position);
  state.update();

  collision_detection::DistanceRequest request;
  request.type = collision_detection::DistanceRequestTypes::GLOBAL;
  request.enable_nearest_points = true;
  request.enable_signed_distance = true;
  request.group_name = "arm";
  request.enableGroup(robot_model);
  request.acm = &scene.getAllowedCollisionMatrix();

  collision_detection::DistanceResult result;
  scene.getCollisionEnv()->distanceSelf(request, result, state);

  const auto & nearest = result.minimum_distance;
  RCLCPP_INFO(node->get_logger(), "Nearest self-collision pair: %s <-> %s",
              nearest.link_names[0].c_str(), nearest.link_names[1].c_str());
  RCLCPP_INFO(node->get_logger(), "Signed distance: %.9f m (%.3f mm)",
              nearest.distance, nearest.distance * 1000.0);
  RCLCPP_INFO(node->get_logger(), "Nearest point on %s: [%.6f, %.6f, %.6f]",
              nearest.link_names[0].c_str(), nearest.nearest_points[0].x(),
              nearest.nearest_points[0].y(), nearest.nearest_points[0].z());
  RCLCPP_INFO(node->get_logger(), "Nearest point on %s: [%.6f, %.6f, %.6f]",
              nearest.link_names[1].c_str(), nearest.nearest_points[1].x(),
              nearest.nearest_points[1].y(), nearest.nearest_points[1].z());

  rclcpp::shutdown();
  return 0;
}
