import pytest

from src.constants import VDIFF_MAX_VOLTS
from src.exceptions import UnsafeVoltageRequest
from src.pid_helpers import PidAxisState, PidGains, checked_vdiff_command, pid_axis_step, validate_pid_inputs


def test_validate_pid_inputs_rejects_non_positive_refresh():
    gains = PidGains(kp=1.0, ki=0.0, kd=0.0)
    with pytest.raises(ValueError, match="refresh"):
        validate_pid_inputs(refresh_ms=0.0, gains=gains)


def test_validate_pid_inputs_rejects_non_finite_gain():
    gains = PidGains(kp=float("nan"), ki=0.0, kd=0.0)
    with pytest.raises(ValueError, match="kp"):
        validate_pid_inputs(refresh_ms=500.0, gains=gains)


def test_pid_axis_step_standard_formula():
    gains = PidGains(kp=2.0, ki=1.0, kd=0.5)
    state = PidAxisState(integral=0.0, prev_error=0.0)
    dt = 0.5
    delta, next_state = pid_axis_step(error=1.0, state=state, gains=gains, dt_s=dt)
    # P=2.0, I=1.0*(1*0.5)=0.5, D=0.5*((1-0)/0.5)=1.0
    assert delta == pytest.approx(3.5)
    assert next_state.integral == pytest.approx(0.5)
    assert next_state.prev_error == pytest.approx(1.0)


def test_pid_axis_step_rejects_bad_dt():
    gains = PidGains(kp=1.0, ki=0.0, kd=0.0)
    with pytest.raises(ValueError, match="dt_s"):
        pid_axis_step(error=0.0, state=PidAxisState(), gains=gains, dt_s=0.0)


def test_checked_vdiff_command_in_range():
    out = checked_vdiff_command(current_vdiff=(1.0, -2.0), delta=(0.25, -0.5))
    assert out == pytest.approx((1.25, -2.5))


def test_checked_vdiff_command_raises_when_out_of_range():
    with pytest.raises(UnsafeVoltageRequest, match="vdiff_x"):
        checked_vdiff_command(current_vdiff=(VDIFF_MAX_VOLTS, 0.0), delta=(0.1, 0.0))
