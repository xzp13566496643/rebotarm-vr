# reBot B601-DM + PICO 4 Pro VR teleoperation handoff

## Environment

- Ubuntu 22.04, ROS 2 Humble.
- Robot: reBot B601-DM, seven Damiao motors, USB2CAN serial adapter.
- Serial device may enumerate as `/dev/ttyACM0` or `/dev/ttyACM1`; check it on every boot.
- PICO application publishes controller data over UDP through OpenArmX.

## Repositories and pinned revisions

- This repository: based on `Seeed-Projects/reBotArmController_ROS2`, base commit
  `e134941` before the local hardware/VR changes.
- Python hardware SDK (clone separately into `third_party/reBotArm_control_py`):
  `https://github.com/Seeed-Projects/reBotArm_control_py.git`, commit
  `5ba28acef46237eb6a7560658bbc43b06cf8a259`.
- VR bridge workspace source (clone separately):
  `https://github.com/openarmx/openarmx_teleop_vr.git`, branch `6.0_basic`, commit
  `a3da7411b3d6ecaa7f94df859e07fb642aec859b`.
- Hugging Face LeRobot checkout is not required for the current ROS/VR control path.

## Implemented control chain

`PICO controller -> OpenArmX UDP bridge -> vr_to_servo -> MoveIt Servo ->
/rebotarm/servo_joint_trajectory -> reBotArmController -> Damiao motors`

- Left trigger is the arm clutch.
- Right-controller translation commands `gripper_tcp` XYZ in `base_link`.
- Right-controller relative rotation commands TCP roll/pitch/yaw.
- Right trigger independently commands the gripper.
- MoveIt Servo publishes joint positions and velocities.
- The hardware adapter adds bounded velocity lookahead before DM POS_VEL commands.
- Normal `/rebotarm/joint_states` feedback remains active; the redundant synchronous
  CAN read on every Servo command is skipped when `servo_max_step == 0`.
- Collision exclusions added for `link5 <-> gripper_link` and
  `link4 <-> gripper_link` after measured false proximity from the mesh geometry.

## Important current parameters

- TCP max linear speed: `0.10 m/s`.
- TCP max angular speed: `0.60 rad/s`.
- Arm motor Servo velocity ceiling: `1.0 rad/s`.
- Velocity lookahead: `0.15 s`, capped to `0.05 rad` per joint.
- VR gripper POS_VEL limit: `2.0 rad/s`.
- Configured gripper endpoints are currently `close=0`, `open=-5 rad`; these must
  be verified against the DMTool calibration before using the full range.

## Safety-critical unresolved issue

Do not use `/rebotarm/safe_home` until its gripper behavior is fixed.
`safe_home()` currently calls `set_gripper_position(0.0)`, which uses MIT position
control (`kp=8`, `kd=1`) without a velocity limit. It can close the gripper very
quickly and is independent of the VR `2.0 rad/s` setting.

The VR gripper path also lacks application-level position clamping, force/stall
stop, and low-force object holding. Torque feedback is published on
`/rebotarm/gripper/state`, but the VR POS_VEL command does not consume it. Do not
test the gripper against an object or mechanical stop until these protections are
implemented.

The gripper was recalibrated with Windows DMTool after an impact. Do not call the
ROS `/rebotarm/set_zero` service unless the exact official mechanical calibration
pose has been confirmed. The current single-joint zero implementation disables
all motors first, so an unsupported arm can fall.

## Key local files

- `src/command/src/vr_to_servo.cpp`: PICO mapping, clutch, TCP twist, gripper command.
- `src/command/config/servo_hardware.yaml`: real MoveIt Servo parameters.
- `src/command/launch/vr_servo_hardware.launch.py`: real VR Servo launch.
- `src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py`: Servo trajectory adapter.
- `src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py`: hardware streaming and safe-home behavior.
- `src/rebotarm_bringup/launch/driver.launch.py`: real driver defaults.
- `src/rebotarm_moveit_config/config/rebotarm.srdf`: collision exclusions.

## Build

```bash
cd /path/to/rebotarm_ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Build the OpenArmX bridge separately in a ROS workspace and source its install
space before starting the bridge.

See the external Chinese operation notes in `~/ws/lerobot/操作.md` on the original
computer. Copy them into the repository before handoff if desired, but note that
the normal-shutdown section still needs updating to avoid the unsafe current
`safe_home` gripper close.
