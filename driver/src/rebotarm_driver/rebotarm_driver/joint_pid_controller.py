"""机械臂六关节上位机 PID/PD 跟踪控制器。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class JointPidOutput:
    """保存一次关节 PID 计算生成的 POS_VEL 命令。"""

    positions: np.ndarray
    velocity_limits: np.ndarray
    commanded_velocities: np.ndarray
    position_errors: np.ndarray


class JointPidController:
    """用关节位置反馈修正 Servo 目标，并生成下一周期 POS_VEL 命令。

    控制公式为：

        qd_cmd = qd_target + kp * e + ki * integral(e) + kd * de/dt
        q_cmd = q_actual + qd_cmd * dt

    其中 ``e = q_target - q_actual``。若调用方提供真实关节速度，
    ``de/dt`` 使用 ``qd_target - qd_actual``；否则使用相邻两次位置误差差分。

    该类只做数值计算，不创建 ROS 订阅、不读取硬件，也不发送电机命令。
    """

    def __init__(
        self,
        joint_names: list[str],
        kp: float | list[float] | np.ndarray,
        ki: float | list[float] | np.ndarray,
        kd: float | list[float] | np.ndarray,
        velocity_limits: float | list[float] | np.ndarray,
        acceleration_limits: float | list[float] | np.ndarray,
        integral_limits: float | list[float] | np.ndarray,
    ) -> None:
        """保存六关节增益和限制；不会启动控制或访问真机。"""
        if not joint_names:
            raise ValueError("joint_names不能为空")
        if len(set(joint_names)) != len(joint_names):
        
            raise ValueError("joint_names不能包含重复名称")

        self.joint_names = list(joint_names)
        self._size = len(self.joint_names)
        self.kp = self._as_vector(kp, "kp", nonnegative=True)
        self.ki = self._as_vector(ki, "ki", nonnegative=True)
        self.kd = self._as_vector(kd, "kd", nonnegative=True)
        self.velocity_limits = self._as_vector(
            velocity_limits, "velocity_limits", positive=True
        )
        self.acceleration_limits = self._as_vector(
            acceleration_limits, "acceleration_limits", positive=True
        )
        self.integral_limits = self._as_vector(
            integral_limits, "integral_limits", nonnegative=True
        )

        self._integral_error = np.zeros(self._size, dtype=np.float64)
        self._previous_error: np.ndarray | None = None
        self._previous_velocity_command = np.zeros(self._size, dtype=np.float64)

    def reset(self) -> None:
        """清除积分、上一周期误差和速度命令，供离合松开或控制重启时调用。"""
        self._integral_error.fill(0.0)
        self._previous_error = None
        self._previous_velocity_command.fill(0.0)

    def compute(
        self,
        target_positions: list[float] | np.ndarray,
        actual_positions: list[float] | np.ndarray,
        dt: float,
        target_velocities: list[float] | np.ndarray | None = None,
        actual_velocities: list[float] | np.ndarray | None = None,
    ) -> JointPidOutput:
        """根据最新目标和反馈计算一次位置命令与正速度上限。

        Args:
            target_positions: MoveIt Servo 给出的关节目标位置，单位 rad。
            actual_positions: 真机关节反馈位置，单位 rad。
            dt: 本次与上次计算的时间间隔，单位 s。
            target_velocities: Servo 给出的关节目标速度，单位 rad/s；可省略。
            actual_velocities: 真机反馈速度，单位 rad/s；可省略。

        Returns:
            可直接转换为 ``send_pos_vel(position, vlim)`` 输入的数据。
        """
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt必须是有限正数，当前为{dt!r}")

        target = self._as_state(target_positions, "target_positions")
        actual = self._as_state(actual_positions, "actual_positions")
        target_velocity = (
            np.zeros(self._size, dtype=np.float64)
            if target_velocities is None
            else self._as_state(target_velocities, "target_velocities")
        )

        error = target - actual
        self._integral_error = np.clip(
            self._integral_error + error * dt,
            -self.integral_limits,
            self.integral_limits,
        )

        if actual_velocities is not None:
            actual_velocity = self._as_state(actual_velocities, "actual_velocities")
            error_derivative = target_velocity - actual_velocity
        elif self._previous_error is None:
            error_derivative = np.zeros(self._size, dtype=np.float64)
        else:
            error_derivative = (error - self._previous_error) / dt

        requested_velocity = (
            target_velocity
            + self.kp * error
            + self.ki * self._integral_error
            + self.kd * error_derivative
        )

        # 先限制相邻周期速度变化，再限制绝对速度，避免PID输出发生阶跃。
        maximum_velocity_change = self.acceleration_limits * dt
        commanded_velocity = np.clip(
            requested_velocity,
            self._previous_velocity_command - maximum_velocity_change,
            self._previous_velocity_command + maximum_velocity_change,
        )
        commanded_velocity = np.clip(
            commanded_velocity,
            -self.velocity_limits,
            self.velocity_limits,
        )

        command_position = actual + commanded_velocity * dt
        self._previous_error = error.copy()
        self._previous_velocity_command = commanded_velocity.copy()

        return JointPidOutput(
            positions=command_position,
            velocity_limits=np.abs(commanded_velocity),
            commanded_velocities=commanded_velocity,
            position_errors=error,
        )

    def _as_vector(
        self,
        value: float | list[float] | np.ndarray,
        label: str,
        *,
        nonnegative: bool = False,
        positive: bool = False,
    ) -> np.ndarray:
        """把标量扩展成每关节参数，或验证已提供的逐关节参数。"""
        array = np.asarray(value, dtype=np.float64)
        if array.ndim == 0:
            array = np.full(self._size, float(array), dtype=np.float64)
        else:
            array = array.reshape(-1)
        if array.shape != (self._size,) or not np.all(np.isfinite(array)):
            raise ValueError(f"{label}必须是一个有限标量或长度为{self._size}的数组")
        if positive and np.any(array <= 0.0):
            raise ValueError(f"{label}中的值必须大于0")
        if nonnegative and np.any(array < 0.0):
            raise ValueError(f"{label}中的值不能小于0")
        return array.copy()

    def _as_state(
        self,
        value: list[float] | np.ndarray,
        label: str,
    ) -> np.ndarray:
        """验证一次六关节目标或反馈向量。"""
        array = np.asarray(value, dtype=np.float64).reshape(-1)
        if array.shape != (self._size,) or not np.all(np.isfinite(array)):
            raise ValueError(f"{label}必须包含{self._size}个有限数值")
        return array
