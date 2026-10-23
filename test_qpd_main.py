"""
Basic PID test with QPD. All commands go through FSM object. In order to get
the qpd working on the raspberry pi, you need to ensure: 
- I2C 1 (or whatever the first I2C is) is enabled
- IMPORTANT!!!!: The DAC I2C pullups pull it up to 3.3V, and NOT 5V, the 
raspberry pi gpio pins cannot handle 5V!!!
- the actual pins from the dac are connected to the i2c pins on the raspberry pi

Raspberry pi i2c physical pins should be..
- SDA1: Physical Pin #3, BCM Pin #2
- SCL1: Physical Pin #5, BCM Pin #3
(so connect DAC SDA to pin 3, SCL to pin 5, and then DAC GND to rpi GND and dac Vin to rpi +3.3V)


HOW THIS FILE WORKS: 

This is just an infinite loop with some settings that asks you for the PID
constants: kp (proportional), kd (derivative), ki (integral). If k* = 0, that
part of the equation is not used. 

Once you give it the constants when you run "python3 test_qpd_main.py -k* #", 
it reads out the voltages from the qpd and calculates a normalized coordinate: This is
where the laser currently is on the qpd. What's cool about PID is that this normalized 
coordinate doesn't really have to have any physical meaning at all -- as long as we 
do all the math in that coordinate system we will be able to have a working pid loop. 

All of the following is done in the function pid_axis_step(): 
Once we have that coordinate, we calculate error = setpoint - laser_location, where the
setpoint is just the center of the qpd (so at (0, 0)). We feed that error into the
pid formula, where delta V = kp * error + kd * d/dt(error) + ki * integral(error). 

We then add the delta on each axis to the current fsm position, wait for it to update,
and repeat all of the above. Again, its super cool because we don't really need 
a voltage-angle mapping at all, everything is based on how we tune the coefficients
ki, kp, kd. 

All voltage commands are written through the FSM() object, which has a bunch of safety
checks so you don't break shit. There are also other safety checks in this file so
nothing SHOULD break, but be careful. 
"""

from __future__ import annotations


import time
from pathlib import Path
from typing import Any

import click

from src import FSM
from src import constants
from src.exceptions import HardwareUnavailable, UnsafeVoltageRequest

from src.pid_helpers import PidAxisState, PidGains, checked_vdiff_command, pid_axis_step, validate_pid_inputs
from src.adc_raw import get_centroid_err_qpd, adc_init, adc_close



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
):
    fsm = FSM()
    adc = adc_init()

    try:
        gains = PidGains(kp=kp, ki=ki, kd=kd)
        validate_pid_inputs(refresh_ms=refresh_ms, gains=gains)


        result = fsm.begin_interactive()
        if not result.ok:
            _safe_stop_fsm(fsm, reason="FSM failed to start or was aborted.")
            adc_close(adc)
            return 1


        click.echo(f"PID started in {mode} mode. refresh={refresh_ms} ms gains=({kp}, {ki}, {kd})")

        state_x = PidAxisState()
        state_y = PidAxisState()
        iteration = 0

        # timing states
        dt_s = refresh_ms / 1000.0
        prev_sample_time = None



        while True:

            t_cycle_s = time.perf_counter()

            if mode == "step":
                key = input("Press Enter for next PID step (q to quit): ").strip().lower()
                if key in {"q", "quit", "exit"}:
                    click.echo("Step mode exit requested.")
                    break
            read_s = time.perf_counter()
            x_err_raw, y_err_raw = get_centroid_err_qpd(adc)
            read_e = time.perf_counter()
            err_x = float(setpoint_x) - float(x_err_raw)
            err_y = float(setpoint_y) - float(y_err_raw)

            # more timing
            sample_time = (read_e - read_s) / 2
            if prev_sample_time is None:
                dt_actual = dt_s
            else:
                dt_actual = sample_time - prev_sample_time
            prev_sample_time = sample_time

            delta_x, state_x = pid_axis_step(error=err_x, state=state_x, gains=gains, dt_s=dt_actual)
            delta_y, state_y = pid_axis_step(error=err_y, state=state_y, gains=gains, dt_s=dt_actual)

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

            iteration += 1

            if mode == "auto":
                remaining = dt_s - (time.perf_counter() - t_cycle_s)
                if remaining > 0:
                    time.sleep(remaining)

        _safe_stop_fsm(fsm, reason="PID loop ended. Returning to origin.")
        adc_close(adc)
        return 0

    except KeyboardInterrupt:
        _safe_stop_fsm(fsm, reason="KeyboardInterrupt: stopping PID and returning to origin.")
        adc_close(adc)
        return 1
    except (ValueError, UnsafeVoltageRequest, HardwareUnavailable) as exc:
        _safe_stop_fsm(fsm, reason=f"PID error: {exc}")
        adc_close(adc)
        return 1
    except Exception as exc:
        _safe_stop_fsm(fsm, reason=f"Unexpected crash: {exc}")
        adc_close(adc)
        return 1


if __name__ == "__main__":
    cmd()