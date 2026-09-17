#include <memory>

#include "arm_controller.hpp"
#include "grasp_controller.hpp"
#include "rclcpp/rclcpp.hpp"

/** 创建ROS 2节点和机械臂、夹爪两个独立控制对象。 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vr_to_control");

  ArmController arm_controller(*node);
  GraspController grasp_controller(*node);

  RCLCPP_INFO(node->get_logger(),
              "VR控制已启动：左扳机控制机械臂，右扳机控制夹爪");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
