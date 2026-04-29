"""
Basic PID mock runner:
- Initializes FSM and QPD camera server.
- Streams centroid error from src.picam_qpd_centroid.
- Applies PID delta corrections onto current vdiff.
"""

from __future__ import annotations

import csv
import json
import re
import time
from pathlib import Path
from typing import Any

import click

from src import FSM
from src import constants
from src.exceptions import HardwareUnavailable, UnsafeVoltageRequest
from src.picam_qpd_centroid import (
    FRAME_HEIGHT,
    FRAME_RATE,
    FRAME_WIDTH,
    get_centroid_err,
    start_camera_server,
)
from src.pid_helpers import PidAxisState, PidGains, checked_vdiff_command, pid_axis_step, validate_pid_inputs


def _resolve_pid_csv_path(out: Path) -> Path:
    """If ``out`` ends with ``.csv``, use that file; otherwise treat as a directory and pick ``testN.csv``."""
    if out.suffix.lower() == ".csv":
        out.parent.mkdir(parents=True, exist_ok=True)
        return out
    out.mkdir(parents=True, exist_ok=True)
    used: list[int] = []
    for p in out.iterdir():
        m = re.fullmatch(r"test(\d+)\.csv", p.name, flags=re.IGNORECASE)
        if m:
            used.append(int(m.group(1)))
    n = max(used, default=0) + 1
    return out / f"test{n}.csv"


def _write_pid_csv_metadata(
    fh,
    *,
    refresh_ms: float,
    kp: float,
    ki: float,
    kd: float,
    setpoint_x: float,
    setpoint_y: float,
    mode: str,
    resolution_cli: int,
    dt_s: float,
    fsm: FSM,
    csv_path: Path,
) -> None:
    rows = {
        "csv_path": str(csv_path),
        "refresh_ms": refresh_ms,
        "dt_s": dt_s,
        "kp": kp,
        "ki": ki,
        "kd": kd,
        "setpoint_x": setpoint_x,
        "setpoint_y": setpoint_y,
        "mode": mode,
        "resolution_cli_width": resolution_cli,
        "camera_frame_width": FRAME_WIDTH,
        "camera_frame_height": FRAME_HEIGHT,
        "camera_frame_rate_fps": FRAME_RATE,
        "fsm_slew_time_ms": fsm.slew_time,
        "fsm_slew_step_v": fsm.slew_step,
        "vdiff_min_v": constants.VDIFF_MIN_VOLTS,
        "vdiff_max_v": constants.VDIFF_MAX_VOLTS,
        "vbias_v": constants.VBIAS,
    }
    for key, val in rows.items():
        fh.write(f"# {key}={val!r}\n")
    fh.write(
        "# camera_error_* = raw x,y from get_centroid_err(); "
        "pid_error_* = setpoint - camera (PID loop error)\n"
    )


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
@click.option(
    "--o",
    "out_csv",
    type=click.Path(path_type=Path, dir_okay=True, file_okay=True),
    default=None,
    help=(
        "CSV log: pass a .csv file path, or a directory (e.g. mock_pid_out) for auto-named testN.csv. "
        "Omit to disable logging."
    ),
)
def cmd(
    refresh_ms: float,
    kp: float,
    ki: float,
    kd: float,
    setpoint_x: float,
    setpoint_y: float,
    mode: str,
    resolution: int,
    out_csv: Path | None,
):
    fsm = FSM()

    try:
        gains = PidGains(kp=kp, ki=ki, kd=kd)
        validate_pid_inputs(refresh_ms=refresh_ms, gains=gains)
        dt_s = refresh_ms / 1000.0

        result = fsm.begin_interactive()
        if not result.ok:
            _safe_stop_fsm(fsm, reason="FSM failed to start or was aborted.")
            return 1

        try:
            start_camera_server()
        except Exception as exc:
            raise HardwareUnavailable(f"Failed to start QPD camera server: {exc}") from exc

        # #region agent log
        try:
            _rec = {
                "sessionId": "851f46",
                "timestamp": int(time.time() * 1000),
                "location": "pid_mock_main.py:cmd",
                "message": "camera_server_started",
                "data": {"mode": mode, "kp": kp},
                "hypothesisId": "H1",
            }
            with open("/home/ctebow/pulse-a/mirrorctl/.cursor/debug-851f46.log", "a", encoding="utf-8") as _f:
                _f.write(json.dumps(_rec) + "\n")
        except Exception:
            pass
        # #endregion

        click.echo(f"PID started in {mode} mode. refresh={refresh_ms} ms gains=({kp}, {ki}, {kd})")

        state_x = PidAxisState()
        state_y = PidAxisState()
        iteration = 0

        csv_fh = None
        csv_writer: Any = None
        if out_csv is not None:
            csv_path_resolved = _resolve_pid_csv_path(out_csv)
            csv_fh = open(csv_path_resolved, "w", newline="", encoding="utf-8")
            try:
                _write_pid_csv_metadata(
                    csv_fh,
                    refresh_ms=refresh_ms,
                    kp=kp,
                    ki=ki,
                    kd=kd,
                    setpoint_x=setpoint_x,
                    setpoint_y=setpoint_y,
                    mode=mode,
                    resolution_cli=resolution,
                    dt_s=dt_s,
                    fsm=fsm,
                    csv_path=csv_path_resolved,
                )
                csv_writer = csv.writer(csv_fh)
                csv_writer.writerow(
                    [
                        "iteration",
                        "curr_vdiff_x",
                        "curr_vdiff_y",
                        "camera_error_x",
                        "camera_error_y",
                        "pid_error_x",
                        "pid_error_y",
                        "pid_delta_x",
                        "pid_delta_y",
                        "new_vdiff_x",
                        "new_vdiff_y",
                    ]
                )
            except Exception:
                try:
                    csv_fh.close()
                except OSError:
                    pass
                csv_fh = None
                csv_writer = None
                raise
            click.echo(f"CSV logging to {csv_path_resolved}")

        try:
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
                fsm.set_vdiff_pid(cmd_vx, cmd_vy)
                click.echo(
                    f"[{iteration}] err=({err_x:.6f},{err_y:.6f}) "
                    f"delta=({delta_x:.6f},{delta_y:.6f}) cmd=({cmd_vx:.6f},{cmd_vy:.6f})"
                )
                if csv_writer is not None:
                    csv_writer.writerow(
                        [
                            iteration,
                            f"{curr_vx:.9f}",
                            f"{curr_vy:.9f}",
                            f"{float(x_err_raw):.9f}",
                            f"{float(y_err_raw):.9f}",
                            f"{err_x:.9f}",
                            f"{err_y:.9f}",
                            f"{delta_x:.9f}",
                            f"{delta_y:.9f}",
                            f"{cmd_vx:.9f}",
                            f"{cmd_vy:.9f}",
                        ]
                    )
                iteration += 1

                if mode == "auto":
                    time.sleep(dt_s)
        finally:
            if csv_fh is not None:
                try:
                    csv_fh.close()
                except OSError:
                    pass

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


if __name__ == "__main__":
    cmd()
