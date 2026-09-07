# reBotArm B601-DM + PICO VR 控制

本仓库只保留当前实际使用的两条流程：

- MoveIt 2 + RViz 仿真中的 PICO VR 累计位姿控制；
- reBot B601-DM 真机的 PICO VR 累计位姿控制。

运行环境：Ubuntu 22.04、ROS 2 Humble、MoveIt 2、PICO 4 Pro、USB2CAN。

## 控制方式

- 左手扳机：按住启用机械臂跟随，松开停止跟随。
- 右手柄位姿：控制 `gripper_tcp` 的位置和姿态。
- 右手扳机：控制夹爪打开和关闭。
- 机械臂使用 MoveIt Servo，保留碰撞、奇异点和关节限位保护。
- 真机操作文档使用 `grasp_mit`：闭合前馈力矩 `0.5 Nm`，开爪 `-1.0 Nm`，检测接触后保持前馈力矩 `0.30 Nm`；打开位置 `-4.60 rad`。启动时须指定 `gripper_control_mode:=grasp_mit`。

```text
PICO OpenArmX应用
  → UDP 5100
  → openarmx_teleop_bridge_vr
  → vr_pose_target_servo
  → MoveIt Servo
  → reBotArmController
  → USB2CAN
  → 机械臂和夹爪
```

## 新电脑安装

先安装 ROS 2 Humble 和 MoveIt 2，并在 PICO 中安装、配置 OpenArmX 发送端。

```bash
mkdir -p "$HOME/ws"
cd "$HOME/ws"
git clone https://github.com/xzp13566496643/rebotarm-vr.git
cd rebotarm-vr

chmod +x scripts/setup_vr_workspace.sh
./scripts/setup_vr_workspace.sh
```

准备脚本会自动下载固定版本的底层机械臂 SDK 和 PICO ROS 桥接、安装依赖，并编译两个工作空间。

如果是第一次使用 rosdep，先执行：

```bash
sudo rosdep init
rosdep update
```

安装后验证：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/ws/rebotarm-vr/install/setup.bash"
source "$HOME/ws/rebotarm-vr/vr_ws/install/setup.bash"

ros2 pkg prefix rebotarm_command
ros2 pkg prefix rebotarmcontroller
ros2 pkg prefix openarmx_teleop_bridge_vr
```

## 操作文档

- [真机和仿真完整操作步骤](位姿控制的真机和仿真.md)
- [原开发电脑操作命令](操作.md)（含本机绝对路径，新电脑请使用上面的通用步骤）
- [项目工作流程报告](项目工作流程报告.md)

开始真机操作前必须先阅读完整操作步骤，不要只复制单条命令。

## 保留的 ROS 2 包

```text
src/command
    PICO累计位姿映射、夹爪MIT控制和MoveIt Servo入口

src/rebotarm_bringup
    真机驱动启动、URDF和硬件配置

src/rebotarm_moveit_config
    MoveIt、RViz、规划组、碰撞和关节限制

src/rebotarm_msgs
    真机驱动使用的消息、服务和动作定义

src/rebotarmcontroller
    轨迹、低层命令、状态反馈、使能和失能
```

## 重要安全说明

- 机械臂悬空时不要直接失能或断电。
- 真机启动 Servo 前，先移动到文档中的安全起始位姿。
- 电机出现 ACK 超时、模式切换失败或 `status=None` 时不要继续。
- 当前不要使用 `/rebotarm/safe_home`，它会同时控制夹爪。
- 夹爪已加入基于位置/速度的接触判断；尚无驱动器总力矩硬限幅，检测也不能保证不压坏物体，禁止撞击机械限位。
- 紧急情况下先托稳机械臂，再使用急停或切断主电源。
