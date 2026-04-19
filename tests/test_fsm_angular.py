import math

import pytest

from src.exceptions import UnsafeAngleRequest, UnsafeVoltageRequest
from src.fsm_angular import AngularSafetyLimits, AxisPolynomialMap, FsmAngular


class StubFSM:
    def __init__(self):
        self.vdiff_x = 0.0
        self.vdiff_y = 0.0
        self.calls: list[tuple[float, float]] = []
        self.raise_on_set: Exception | None = None

    def set_vdiff(self, vdiff_x_new: float, vdiff_y_new: float):
        if self.raise_on_set is not None:
            raise self.raise_on_set
        self.vdiff_x = vdiff_x_new
        self.vdiff_y = vdiff_y_new
        self.calls.append((vdiff_x_new, vdiff_y_new))
        return (self.vdiff_x, self.vdiff_y)

    def get_voltages(self):
        return (self.vdiff_x, self.vdiff_y)

    def begin(self, **_kwargs):
        return {"ok": True}

    def close(self):
        self.vdiff_x = 0.0
        self.vdiff_y = 0.0

    def is_active(self):
        return True


def make_wrapper(stub_fsm: StubFSM) -> FsmAngular:
    # vdiff = 100 * theta_rad for both axes.
    x_poly = AxisPolynomialMap(coefficients=(100.0, 0.0))
    y_poly = AxisPolynomialMap(coefficients=(100.0, 0.0))
    limits = AngularSafetyLimits.from_degrees(max_abs_x_deg=45.0, max_abs_y_deg=45.0, max_radius_deg=60.0)
    return FsmAngular(stub_fsm, x_poly=x_poly, y_poly=y_poly, angle_limits=limits)


def test_set_angle_deg_maps_to_vdiff():
    stub = StubFSM()
    ctrl = make_wrapper(stub)
    out = ctrl.set_angle_deg(10.0, -5.0)
    expected_x = 100.0 * math.radians(10.0)
    expected_y = 100.0 * math.radians(-5.0)
    assert out == (expected_x, expected_y)
    assert stub.calls[-1] == (expected_x, expected_y)


def test_set_angle_rad_matches_deg_equivalent():
    stub = StubFSM()
    ctrl = make_wrapper(stub)
    rad_x = math.radians(12.0)
    rad_y = math.radians(-3.0)
    ctrl.set_angle_rad(rad_x, rad_y)
    got = stub.calls[-1]
    assert got == (100.0 * rad_x, 100.0 * rad_y)


def test_absolute_target_not_incremental():
    stub = StubFSM()
    ctrl = make_wrapper(stub)
    first = ctrl.set_angle_deg(10.0, 10.0)
    second = ctrl.set_angle_deg(10.0, 10.0)
    assert first == second
    assert len(stub.calls) == 2
    assert stub.calls[0] == stub.calls[1]


def test_out_of_range_angle_rejected_before_fsm_command():
    stub = StubFSM()
    ctrl = make_wrapper(stub)
    with pytest.raises(UnsafeAngleRequest, match="theta_x"):
        ctrl.set_angle_deg(50.0, 0.0)
    assert stub.calls == []


def test_non_finite_angle_rejected():
    stub = StubFSM()
    ctrl = make_wrapper(stub)
    with pytest.raises(UnsafeAngleRequest, match="finite"):
        ctrl.set_angle(float("nan"), 0.0, unit="rad")


def test_mapped_vdiff_out_of_range_rejected_before_fsm_command():
    stub = StubFSM()
    # Intentionally aggressive mapping to exceed vdiff range even for small angles.
    x_poly = AxisPolynomialMap(coefficients=(5000.0, 0.0))
    y_poly = AxisPolynomialMap(coefficients=(100.0, 0.0))
    limits = AngularSafetyLimits.from_degrees(max_abs_x_deg=45.0, max_abs_y_deg=45.0)
    ctrl = FsmAngular(stub, x_poly=x_poly, y_poly=y_poly, angle_limits=limits)
    with pytest.raises(UnsafeVoltageRequest, match="Mapped vdiff_x"):
        ctrl.set_angle_deg(5.0, 0.0)
    assert stub.calls == []


def test_underlying_fsm_safety_exception_propagates():
    stub = StubFSM()
    stub.raise_on_set = UnsafeVoltageRequest("base blocked", axis="x")
    ctrl = make_wrapper(stub)
    with pytest.raises(UnsafeVoltageRequest, match="base blocked"):
        ctrl.set_angle_deg(1.0, 1.0)


def test_get_target_angle_returns_last_commanded():
    stub = StubFSM()
    ctrl = make_wrapper(stub)
    ctrl.set_angle_deg(9.0, -4.0)
    assert ctrl.get_target_angle(unit="deg") == pytest.approx((9.0, -4.0))
    assert ctrl.get_target_angle(unit="rad") == pytest.approx((math.radians(9.0), math.radians(-4.0)))


def test_distance_api_is_deferred():
    stub = StubFSM()
    ctrl = make_wrapper(stub)
    with pytest.raises(NotImplementedError, match="deferred"):
        ctrl.set_distance(1.0, 1.0)


def test_lut_mode_reserved_for_future():
    stub = StubFSM()
    x_poly = AxisPolynomialMap(coefficients=(1.0, 0.0))
    y_poly = AxisPolynomialMap(coefficients=(1.0, 0.0))
    limits = AngularSafetyLimits.from_degrees(max_abs_x_deg=10.0, max_abs_y_deg=10.0)
    with pytest.raises(NotImplementedError, match="LUT3D"):
        FsmAngular(stub, x_poly=x_poly, y_poly=y_poly, angle_limits=limits, mapping_mode="lut3d")
