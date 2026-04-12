#!/usr/bin/env python3
"""
Mirrorcle MEMS Digital Driver — Raspberry Pi + Python control script.

TRIPLE CHECK BEFORE RUNNING WITH ACTUAL MEMS MIRROR.
VDIFF (Channel 0 - Channel 1 or Channel 2 - Channel 3) MUST NOT EXCEED 2 * VBIAS
(e.g. 180 V for VBIAS=90). Exceeding this can permanently damage the mirror.
"""
from src import FSM
from src.exceptions import UnsafeVoltageRequest


if __name__ == "__main__":
    fsm = FSM()
    result = fsm.begin_interactive()
    print(fsm.get_voltages())

    if not result.ok:
        print("FSM failed to start or was aborted.")
    elif result.simulated:
        print("FSM in test mode (not on Linux / no hardware drivers).")
        try:
            while True:
                usr = input("Input vdiffx, vdiffy (x y): ")
                lst = usr.split()
                if len(lst) != 2:
                    continue
                x, y = lst[0], lst[1]
                if x and y:
                    try:
                        print(f"setting vdiff: x: {x}, y: {y}")
                        fsm.set_vdiff(float(x), float(y))
                    except UnsafeVoltageRequest as exc:
                        print(f"Unsafe: {exc}")
        except KeyboardInterrupt:
            print("Keyboard interrupt received — shutting down")
        finally:
            fsm.close()
    else:
        try:
            while True:
                usr = input("Input vdiffx, vdiffy (x y): ")
                lst = usr.split()
                if len(lst) != 2:
                    continue
                x, y = lst[0], lst[1]
                if x and y:
                    try:
                        print(f"setting vdiff: x: {x}, y: {y}")
                        fsm.set_vdiff(float(x), float(y))
                    except UnsafeVoltageRequest as exc:
                        print(f"Unsafe: {exc}")
        except KeyboardInterrupt:
            print("Keyboard interrupt received — shutting down")
        finally:
            fsm.close()
