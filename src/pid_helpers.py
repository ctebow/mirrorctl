"""
Basic PID helpers for dual-axis FSM control.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .constants import VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS
from .exceptions import UnsafeVoltageRequest


@dataclass(frozen=True)
class PidGains:
    kp: float
    ki: float
    kd: float


@dataclass(frozen=True)
class PidAxisState:
    integral: float = 0.0
    prev_error: float = 0.0


def validate_pid_inputs(*, refresh_ms: float, gains: PidGains) -> None:
    if not math.isfinite(refresh_ms) or refresh_ms <= 0:
        raise ValueError("PID refresh rate must be a finite positive value in milliseconds.")
    for name, value in (("kp", gains.kp), ("ki", gains.ki), ("kd", gains.kd)):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite.")


def pid_axis_step(*, error: float, state: PidAxisState, gains: PidGains, dt_s: float) -> tuple[float, PidAxisState]:
    if not math.isfinite(error):
        raise ValueError("PID error input must be finite.")
    if not math.isfinite(dt_s) or dt_s <= 0:
        raise ValueError("dt_s must be finite and > 0.")

    integral = state.integral + (error * dt_s)
    derivative = (error - state.prev_error) / dt_s
    delta = (gains.kp * error) + (gains.ki * integral) + (gains.kd * derivative)
    if not math.isfinite(delta):
        raise ValueError("Computed PID delta is not finite.")
    return delta, PidAxisState(integral=integral, prev_error=error)


def checked_vdiff_command(*, current_vdiff: tuple[float, float], delta: tuple[float, float]) -> tuple[float, float]:
    vx_cmd = current_vdiff[0] + delta[0]
    vy_cmd = current_vdiff[1] + delta[1]
    _validate_vdiff(vx_cmd, axis="x")
    _validate_vdiff(vy_cmd, axis="y")
    return (vx_cmd, vy_cmd)


def _validate_vdiff(vdiff: float, *, axis: str) -> None:
    if not math.isfinite(vdiff):
        raise UnsafeVoltageRequest(f"vdiff_{axis} command is not finite.", axis=axis)
    if not (VDIFF_MIN_VOLTS <= vdiff <= VDIFF_MAX_VOLTS):
        raise UnsafeVoltageRequest(
            f"vdiff_{axis}={vdiff} exceeds [{VDIFF_MIN_VOLTS}, {VDIFF_MAX_VOLTS}]",
            axis=axis,
        )
