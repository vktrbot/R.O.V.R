"""Tests for camera PTZ clamping and smoothing."""

from __future__ import annotations

from rovr_common.command_models import CameraTargetCommand
from rovr_common.fake_servo_bus import FakeServoBus

from rovr_camera.control import CameraCore


class LaggingFeedbackBus(FakeServoBus):
    def __init__(self, *args: object, feedback_lag: int = 120, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.feedback_lag = int(feedback_lag)

    def write_positions(self, positions: list[int]) -> None:
        for index, value in enumerate(positions):
            if not self.torque_enabled[index]:
                continue
            command = int(value)
            self.goal_positions[index] = command
            self.positions[index] = max(0, command - self.feedback_lag)
        self._update_loads(positions)
        self._record()


def test_camera_clamps_targets_to_limits() -> None:
    bus = FakeServoBus(servo_ids=[1, 2], initial_positions=[2048, 2048])
    core = CameraCore(
        limits=[(1000, 3000), (1500, 2500)],
        bus=bus,
        ema_alpha=0.5,
        max_relative_target=128,
    )
    core.arm_all()

    core.apply_command(CameraTargetCommand(servos=(3500, 1200)))
    positions = core.tick()

    assert positions[0] < 3000
    assert positions[1] > 1500
    assert core.target_positions == [3000, 1500]


def test_camera_reaches_target_command_with_lagging_feedback() -> None:
    bus = LaggingFeedbackBus(servo_ids=[1, 2], initial_positions=[1000, 1000], feedback_lag=120)
    core = CameraCore(
        limits=[(0, 4095), (0, 4095)],
        bus=bus,
        ema_alpha=0.6,
        max_relative_target=64,
        target_snap_delta=48,
    )
    core.arm_all()

    core.apply_command(CameraTargetCommand(servos=(3000, 1000)))
    positions = [0, 0]
    for _ in range(80):
        positions = core.tick()

    assert core.target_positions == [3000, 1000]
    assert bus.goal_positions[0] == 3000
    assert positions[0] == 2880
