#!/usr/bin/env python3
"""基于 Seeed GraspDriver 思路实现的夹爪 MIT 抓取状态机。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class GripperMitCommand:
    """保存一帧即将发送给夹爪电机的 MIT 五元组。"""

    pos: float
    vel: float
    kp: float
    kd: float
    tau: float


class GripperGraspController:
    """执行“打开、恒力闭合、堵转检测、接触位置保持”的夹爪控制。"""

    WAITING = "waiting_feedback"
    IDLE = "idle"
    OPENING = "opening"
    CLOSING = "closing"
    HOLDING = "holding"
    EMPTY = "empty"
    FEEDBACK_TIMEOUT = "feedback_timeout"

    def __init__(
        self,
        *,
        open_position: float,
        closed_position: float,
        move_kp: float = 5.0,
        move_kd: float = 1.0,
        opening_torque_limit: float = 1.0,
        close_kd: float = 0.5,
        close_torque: float = 0.5,
        hold_torque: float = 0.30,
        tau_max: float = 1.5,
        stall_velocity: float = 0.05,
        startup_distance: float = 0.30,
        hard_stop_distance: float = 0.05,
        arrive_tolerance: float = 0.12,
        stall_samples: int = 1,
        feedback_timeout: float = 0.25,
    ) -> None:
        self.open_position = float(open_position)
        self.closed_position = float(closed_position)
        if self.open_position >= self.closed_position:
            raise ValueError("当前DM夹爪要求 open_position < closed_position")

        self.move_kp = max(0.0, float(move_kp))
        self.move_kd = max(0.0, float(move_kd))
        self.opening_torque_limit = min(
            abs(float(opening_torque_limit)), abs(float(tau_max))
        )
        self.close_kd = max(0.0, float(close_kd))
        self.tau_max = abs(float(tau_max))
        self.close_torque = min(abs(float(close_torque)), self.tau_max)
        self.hold_torque = min(abs(float(hold_torque)), self.tau_max)
        self.stall_velocity = abs(float(stall_velocity))
        self.startup_distance = abs(float(startup_distance))
        self.hard_stop_distance = abs(float(hard_stop_distance))
        self.arrive_tolerance = abs(float(arrive_tolerance))
        self.stall_samples = max(1, int(stall_samples))
        self.feedback_timeout = max(0.02, float(feedback_timeout))

        self.state = self.WAITING
        self.position: float | None = None
        self.velocity: float | None = None
        self.torque: float | None = None
        self.feedback_time: float | None = None
        self.start_position: float | None = None
        self.contact_position: float | None = None
        self.stall_count = 0

    @property
    def initialized(self) -> bool:
        """返回是否已经收到至少一帧有效夹爪反馈。"""
        return self.position is not None

    def update_feedback(self, position: float, velocity: float, torque: float) -> bool:
        """缓存最新反馈；第一帧反馈到达时返回 True。"""
        values = (float(position), float(velocity), float(torque))
        if not all(math.isfinite(value) for value in values):
            return False
        first = not self.initialized
        self.position, self.velocity, self.torque = values
        self.feedback_time = time.monotonic()
        if first:
            self.contact_position = self.position
            self.state = self.IDLE
        return first

    def request_close(self) -> bool:
        """从当前反馈位置开始恒力闭合；已闭合或保持时不重复启动。"""
        if not self.initialized or self.state in (self.CLOSING, self.HOLDING):
            return False
        self.start_position = self.position
        self.contact_position = self.position
        self.stall_count = 0
        self.state = self.CLOSING
        return True

    def request_open(self) -> bool:
        """切换到打开位置控制；已经打开时不重复切换。"""
        if not self.initialized or self.state == self.OPENING:
            return False
        self.stall_count = 0
        self.state = self.OPENING
        return True

    def hold_current_command(self) -> GripperMitCommand:
        """生成保持最新反馈位置且不附加前馈力矩的安全初始化命令。"""
        if self.position is None:
            raise RuntimeError("夹爪反馈尚未初始化")
        return GripperMitCommand(
            self.position, 0.0, self.move_kp, self.move_kd, 0.0
        )

    def opening_command(self) -> GripperMitCommand:
        """生成与闭合阶段对称的固定力矩开爪命令。"""
        if self.position is None:
            raise RuntimeError("夹爪反馈尚未初始化")
        # kp=0：不让较大的位置误差产生额外力矩；负 tau 驱动DM夹爪打开。
        return GripperMitCommand(
            self.open_position,
            0.0,
            0.0,
            self.close_kd,
            -self.opening_torque_limit,
        )

    def tick(self) -> GripperMitCommand | None:
        """根据最新反馈推进状态机，并生成当前周期的 MIT 命令。"""
        if not self.initialized:
            return None

        now = time.monotonic()
        if self.feedback_time is None or now - self.feedback_time > self.feedback_timeout:
            if self.state == self.CLOSING:
                self.contact_position = self.position
                self.state = self.FEEDBACK_TIMEOUT
            if self.state == self.FEEDBACK_TIMEOUT:
                return self.hold_current_command()

        if self.state == self.OPENING:
            if abs(self.position - self.open_position) <= self.arrive_tolerance:
                self.state = self.IDLE
                # 已到达安全打开位置，不再持续向打开侧施加力矩。
                return GripperMitCommand(
                    self.position, 0.0, 0.0, self.close_kd, 0.0
                )
            return self.opening_command()

        if self.state == self.CLOSING:
            moved = (
                self.start_position is not None
                and abs(self.position - self.start_position) >= self.startup_distance
            )
            at_hard_stop = (
                self.position >= self.closed_position - self.hard_stop_distance
            )
            stalled = moved and abs(self.velocity) < self.stall_velocity
            self.stall_count = self.stall_count + 1 if stalled else 0

            if moved and at_hard_stop:
                self.state = self.EMPTY
                return GripperMitCommand(
                    self.closed_position, 0.0, self.move_kp, self.move_kd, 0.0
                )
            if self.stall_count >= self.stall_samples:
                self.contact_position = self.position
                self.state = self.HOLDING
                return GripperMitCommand(
                    self.contact_position,
                    0.0,
                    self.move_kp,
                    self.move_kd,
                    self.hold_torque,
                )
            # kp=0：闭合阶段只使用固定且受限的前馈力矩，不追赶0 rad。
            return GripperMitCommand(
                self.closed_position, 0.0, 0.0, self.close_kd, self.close_torque
            )

        if self.state == self.HOLDING:
            return GripperMitCommand(
                self.contact_position,
                0.0,
                self.move_kp,
                self.move_kd,
                self.hold_torque,
            )

        if self.state == self.EMPTY:
            return GripperMitCommand(
                self.closed_position, 0.0, self.move_kp, self.move_kd, 0.0
            )

        return self.hold_current_command()
