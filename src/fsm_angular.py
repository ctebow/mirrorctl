"""
Angular command wrapper around the base voltage-domain FSM.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from .constants import VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS
from .exceptions import UnsafeAngleRequest, UnsafeVoltageRequest
from .fsm_obj import FSM

AngleUnit = Literal["deg", "rad"]
MappingMode = Literal["polynomial", "lut3d"]


@dataclass(frozen=True)
class AxisPolynomialMap:
    """
    Per-axis polynomial mapping from angle in radians to vdiff in volts.

    Coefficients are ordered highest power to constant term, consistent with
    common polynomial conventions.
    """

    coefficients: tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.coefficients:
            raise ValueError("Polynomial coefficients must not be empty.")
        if not all(math.isfinite(c) for c in self.coefficients):
            raise ValueError("Polynomial coefficients must be finite numbers.")

    def evaluate(self, angle_rad: float) -> float:
        """Evaluate polynomial via Horner's method."""
        y = 0.0
        for coeff in self.coefficients:
            y = (y * angle_rad) + coeff
        return float(y)


@dataclass(frozen=True)
class AngularSafetyLimits:
    """
    Angular safety envelope in radians.

    - max_abs_x_rad / max_abs_y_rad: per-axis absolute limits.
    - max_radius_rad: optional radial limit sqrt(x^2 + y^2).
    """

    max_abs_x_rad: float
    max_abs_y_rad: float
    max_radius_rad: float | None = None

    @classmethod
    def from_degrees(
        cls,
        *,
        max_abs_x_deg: float,
        max_abs_y_deg: float,
        max_radius_deg: float | None = None,
    ) -> "AngularSafetyLimits":
        max_radius_rad = None if max_radius_deg is None else math.radians(max_radius_deg)
        return cls(
            max_abs_x_rad=math.radians(max_abs_x_deg),
            max_abs_y_rad=math.radians(max_abs_y_deg),
            max_radius_rad=max_radius_rad,
        )

    def __post_init__(self) -> None:
        if not (math.isfinite(self.max_abs_x_rad) and self.max_abs_x_rad > 0):
            raise ValueError("max_abs_x_rad must be a finite positive number.")
        if not (math.isfinite(self.max_abs_y_rad) and self.max_abs_y_rad > 0):
            raise ValueError("max_abs_y_rad must be a finite positive number.")
        if self.max_radius_rad is not None and not (
            math.isfinite(self.max_radius_rad) and self.max_radius_rad > 0
        ):
            raise ValueError("max_radius_rad must be None or a finite positive number.")

    def check_or_raise(self, theta_x_rad: float, theta_y_rad: float) -> None:
        if abs(theta_x_rad) > self.max_abs_x_rad:
            raise UnsafeAngleRequest(
                f"theta_x={theta_x_rad} rad exceeds +/-{self.max_abs_x_rad} rad",
                axis="x",
            )
        if abs(theta_y_rad) > self.max_abs_y_rad:
            raise UnsafeAngleRequest(
                f"theta_y={theta_y_rad} rad exceeds +/-{self.max_abs_y_rad} rad",
                axis="y",
            )
        if self.max_radius_rad is not None:
            radius = math.hypot(theta_x_rad, theta_y_rad)
            if radius > self.max_radius_rad:
                raise UnsafeAngleRequest(
                    f"angular radius={radius} rad exceeds {self.max_radius_rad} rad",
                    axis="xy",
                )


class FsmAngular:
    """
    Safe angular control layer that wraps the base `FSM`.

    Inputs are absolute angular targets from origin. Internally the wrapper maps
    angle->vdiff via per-axis polynomial models and delegates actuation to
    `FSM.set_vdiff(...)` as the final safety gate.
    """

    def __init__(
        self,
        fsm: FSM,
        *,
        x_poly: AxisPolynomialMap,
        y_poly: AxisPolynomialMap,
        angle_limits: AngularSafetyLimits,
        mapping_mode: MappingMode = "polynomial",
    ):
        if mapping_mode != "polynomial":
            raise NotImplementedError(
                "Only mapping_mode='polynomial' is currently implemented. "
                "LUT3D support is reserved for a future release."
            )
        self._fsm = fsm
        self._x_poly = x_poly
        self._y_poly = y_poly
        self._angle_limits = angle_limits
        self._mapping_mode = mapping_mode
        self._target_x_rad = 0.0
        self._target_y_rad = 0.0

    @property
    def fsm(self) -> FSM:
        return self._fsm

    def begin(self, **kwargs):
        return self._fsm.begin(**kwargs)

    def close(self) -> None:
        self._fsm.close()
        self._target_x_rad = 0.0
        self._target_y_rad = 0.0

    def is_active(self) -> bool:
        return self._fsm.is_active()

    def get_voltages(self) -> tuple[float, float]:
        return self._fsm.get_voltages()

    def get_target_angle(self, unit: AngleUnit = "deg") -> tuple[float, float]:
        if unit == "rad":
            return (self._target_x_rad, self._target_y_rad)
        if unit == "deg":
            return (math.degrees(self._target_x_rad), math.degrees(self._target_y_rad))
        raise ValueError("unit must be 'deg' or 'rad'")

    def set_angle_deg(self, theta_x_deg: float, theta_y_deg: float) -> tuple[float, float]:
        return self.set_angle(theta_x_deg, theta_y_deg, unit="deg")

    def set_angle_rad(self, theta_x_rad: float, theta_y_rad: float) -> tuple[float, float]:
        return self.set_angle(theta_x_rad, theta_y_rad, unit="rad")

    def set_angle(self, theta_x: float, theta_y: float, unit: AngleUnit = "deg") -> tuple[float, float]:
        theta_x_rad, theta_y_rad = self._to_radians(theta_x, theta_y, unit=unit)
        self._check_finite_angle(theta_x_rad, theta_y_rad)
        self._angle_limits.check_or_raise(theta_x_rad, theta_y_rad)

        vdiff_x = self._x_poly.evaluate(theta_x_rad)
        vdiff_y = self._y_poly.evaluate(theta_y_rad)
        self._check_mapped_vdiff(vdiff_x, axis="x")
        self._check_mapped_vdiff(vdiff_y, axis="y")

        result = self._fsm.set_vdiff(vdiff_x, vdiff_y)
        self._target_x_rad = theta_x_rad
        self._target_y_rad = theta_y_rad
        return result

    def set_distance(self, *_args, **_kwargs) -> tuple[float, float]:
        raise NotImplementedError(
            "Distance-based commands are intentionally deferred. "
            "Use set_angle_deg/set_angle_rad for now."
        )

    def _to_radians(self, theta_x: float, theta_y: float, *, unit: AngleUnit) -> tuple[float, float]:
        if unit == "rad":
            return (float(theta_x), float(theta_y))
        if unit == "deg":
            return (math.radians(theta_x), math.radians(theta_y))
        raise ValueError("unit must be 'deg' or 'rad'")

    def _check_finite_angle(self, theta_x_rad: float, theta_y_rad: float) -> None:
        if not math.isfinite(theta_x_rad):
            raise UnsafeAngleRequest("theta_x must be finite.", axis="x")
        if not math.isfinite(theta_y_rad):
            raise UnsafeAngleRequest("theta_y must be finite.", axis="y")

    def _check_mapped_vdiff(self, vdiff: float, *, axis: str) -> None:
        if not math.isfinite(vdiff):
            raise UnsafeVoltageRequest(f"Mapped vdiff_{axis} is not finite.", axis=axis)
        if vdiff < VDIFF_MIN_VOLTS or vdiff > VDIFF_MAX_VOLTS:
            raise UnsafeVoltageRequest(
                f"Mapped vdiff_{axis}={vdiff} not in [{VDIFF_MIN_VOLTS}, {VDIFF_MAX_VOLTS}]",
                axis=axis,
            )
