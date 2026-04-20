"""
Basic PID mock runner:
- Initializes camera and FSM.
- Streams centroid error from raspi_camera stubs.
- Applies PID delta corrections onto current vdiff.
"""

from __future__ import annotations

import time
from typing import Callable

import click

from src import FSM, picam
from src.exceptions import HardwareUnavailable, UnsafeVoltageRequest
from src.pid_helpers import PidAxisState, PidGains, checked_vdiff_command, pid_axis_step, validate_pid_inputs


def _open_camera(resolution: int):
    try:
        return picam.init_camera(resolution)
    except Exception as exc:
        raise HardwareUnavailable(f"Camera failed: {exc}") from exc


def _safe_stop_fsm(fsm: FSM, *, reason: str | None = None) -> None:
    if reason:
        click.echo(reason, err=True)
    try:
        if fsm.is_active():
            fsm.set_vdiff(0.0, 0.0)
    except Exception as exc:  # best effort fallback
        click.echo(f"Failed to set safe origin before shutdown: {exc}", err=True)
    finally:
        fsm.close()


@click.command()
@click.option("--refresh-ms", default=500.0, type=float, help="PID refresh period in milliseconds.")
@click.option("--kp", default=0.0, type=float, help="Proportional gain.")
@click.option("--ki", default=0.0, type=float, help="Integral gain.")
@click.option("--kd", default=0.0, type=float, help="Derivative gain.")
@click.option("--setpoint-x", default=0.0, type=float, help="Setpoint error target for X.")
@click.option("--setpoint-y", default=0.0, type=float, help="Setpoint error target for Y.")
@click.option("--mode", default="auto", type=click.Choice(["auto", "step"]), help="auto or step PID loop mode.")
@click.option("--resolution", default=640, type=int, help="Camera frame width (height 480).")
def cmd(
    refresh_ms: float,
    kp: float,
    ki: float,
    kd: float,
    setpoint_x: float,
    setpoint_y: float,
    mode: str,
    resolution: int,
):
    cam = None
    fsm = FSM()

    try:
        gains = PidGains(kp=kp, ki=ki, kd=kd)
        validate_pid_inputs(refresh_ms=refresh_ms, gains=gains)
        dt_s = refresh_ms / 1000.0

        cam = _open_camera(resolution)

        result = fsm.begin_interactive()
        if not result.ok:
            _safe_stop_fsm(fsm, reason="FSM failed to start or was aborted.")
            return 1

        try:
            from raspi_camera import get_centroid_err, start_camera_server
        except Exception as exc:
            raise HardwareUnavailable(
                f"Failed to import raspi_camera stubs (start_camera_server/get_centroid_err): {exc}"
            ) from exc

        start_camera_server()
        click.echo(f"PID started in {mode} mode. refresh={refresh_ms} ms gains=({kp}, {ki}, {kd})")

        state_x = PidAxisState()
        state_y = PidAxisState()
        iteration = 0

        while True:
            if mode == "step":
                key = input("Press Enter for next PID step (q to quit): ").strip().lower()
                if key in {"q", "quit", "exit"}:
                    click.echo("Step mode exit requested.")
                    break

            x_err_raw, y_err_raw = get_centroid_err()
            err_x = float(setpoint_x) - float(x_err_raw)
            err_y = float(setpoint_y) - float(y_err_raw)

            delta_x, state_x = pid_axis_step(error=err_x, state=state_x, gains=gains, dt_s=dt_s)
            delta_y, state_y = pid_axis_step(error=err_y, state=state_y, gains=gains, dt_s=dt_s)

            curr_vx, curr_vy = fsm.get_voltages()
            cmd_vx, cmd_vy = checked_vdiff_command(
                current_vdiff=(curr_vx, curr_vy),
                delta=(delta_x, delta_y),
            )

            # Base FSM still enforces hard voltage safety in set_vdiff.
            fsm.set_vdiff(cmd_vx, cmd_vy)
            click.echo(
                f"[{iteration}] err=({err_x:.6f},{err_y:.6f}) "
                f"delta=({delta_x:.6f},{delta_y:.6f}) cmd=({cmd_vx:.6f},{cmd_vy:.6f})"
            )
            iteration += 1

            if mode == "auto":
                time.sleep(dt_s)

        _safe_stop_fsm(fsm, reason="PID loop ended. Returning to origin.")
        return 0

    except KeyboardInterrupt:
        _safe_stop_fsm(fsm, reason="KeyboardInterrupt: stopping PID and returning to origin.")
        return 1
    except (ValueError, UnsafeVoltageRequest, HardwareUnavailable) as exc:
        _safe_stop_fsm(fsm, reason=f"PID error: {exc}")
        return 1
    except Exception as exc:
        _safe_stop_fsm(fsm, reason=f"Unexpected crash: {exc}")
        return 1
    finally:
        picam.close_camera(cam)


if __name__ == "__main__":
    cmd()
