# How to use `pid_mock_main.py`

## Basic Info

This script is a PID test runner for the FSM. It starts the FSM, starts the QPD camera server, reads camera position error each loop, computes PID deltas for x and y, and commands new `vdiff` values.

Unlike `voltage_mapping_interactive_main.py`, this file is fully command-line driven with `click` options (no interactive parameter wizard). It is mainly for control-loop tuning and behavior testing, not for generating mapping datasets.

Run it from repo root as:

```bash
python3 -u pid_mock_main.py [OPTIONS]
```

---

## What the control loop actually does

On each loop iteration:

1. Reads camera error from `get_centroid_err()` in `src/picam_qpd_centroid.py`.
2. Computes PID error as:
   - `err_x = setpoint_x - camera_x`
   - `err_y = setpoint_y - camera_y`
3. Computes PID deltas with:
   - `delta = kp*error + ki*integral + kd*derivative`
4. Adds deltas to the current `vdiff` command.
5. Validates that the resulting command stays inside `[VDIFF_MIN_VOLTS, VDIFF_MAX_VOLTS]`.
6. Sends command through `fsm.set_vdiff_pid(...)`.

If any command is unsafe or invalid, it shuts down safely (tries to return to origin, then closes FSM).

---

## Command-line options

All options in `pid_mock_main.py`:

- `--refresh-ms` (default `500.0`)
  - PID loop period in milliseconds.
  - Internally converted to `dt_s = refresh_ms / 1000.0` and used in integral/derivative terms.
  - Must be finite and `> 0`.

- `--kp` (default `0.0`)
  - Proportional gain.

- `--ki` (default `0.0`)
  - Integral gain.

- `--kd` (default `0.0`)
  - Derivative gain.

- `--setpoint-x` (default `0.0`)
  - Target x-error value for control loop.

- `--setpoint-y` (default `0.0`)
  - Target y-error value for control loop.

- `--mode` (default `auto`, choices: `auto`, `step`)
  - `auto`: loop runs continuously, sleeping `dt_s` each cycle.
  - `step`: waits for Enter each cycle; type `q`, `quit`, or `exit` to stop.

- `--resolution` (default `640`)
  - Declared as camera frame width in CLI help.
  - Important nuance: current script passes this only into CSV metadata (`resolution_cli_width`) and does not apply it to camera configuration in `src/picam_qpd_centroid.py`.

- `--o` (optional)
  - CSV logging output target.
  - If value ends in `.csv`, writes exactly to that file.
  - Otherwise treats the value as a directory and auto-creates `testN.csv` (e.g. `test1.csv`, `test2.csv`, ...).
  - If omitted, no CSV is written.

---

## Example usage patterns

### 1) Minimal run (no CSV, all defaults)

```bash
python3 -u pid_mock_main.py
```

### 2) Auto mode with explicit gains

```bash
python3 -u pid_mock_main.py --mode auto --refresh-ms 200 --kp 0.01 --ki 0.0 --kd 0.0
```

### 3) Step mode for safe/manual tuning

```bash
python3 -u pid_mock_main.py --mode step --refresh-ms 500 --kp 0.02 --ki 0.001 --kd 0.0
```

### 4) Write to one fixed CSV file

```bash
python3 -u pid_mock_main.py --mode auto --kp 0.01 --ki 0.0 --kd 0.0 --o runs/pid/mock_run.csv
```

### 5) Auto-number CSV files in a directory

```bash
python3 -u pid_mock_main.py --mode step --kp 0.02 --ki 0.001 --kd 0.0005 --o runs/pid/
```

This will create `runs/pid/test1.csv`, then `test2.csv`, etc.

---

## CSV output format and metadata

If `--o` is provided, the script writes:

1. Metadata comment lines starting with `#`, including:
   - gains, refresh period, setpoints, mode
   - camera dimensions/rate from `src/picam_qpd_centroid.py`
   - FSM slew settings and voltage bounds from constants
2. A data header row:
   - `iteration`
   - `curr_vdiff_x`, `curr_vdiff_y`
   - `camera_error_x`, `camera_error_y`
   - `pid_error_x`, `pid_error_y`
   - `pid_delta_x`, `pid_delta_y`
   - `new_vdiff_x`, `new_vdiff_y`

Important nuance on terms:

- `camera_error_*` are raw values returned by camera (`get_centroid_err()`).
- `pid_error_*` are what PID uses: `setpoint - camera`.
- `curr_vdiff_*` are voltages before this iteration’s update.
- `new_vdiff_*` are the commanded values for this iteration.

---

## How camera error is computed (for this script)

This script does **not** use the mapping centroid pipeline from `src/centroiding.py`.

Instead it uses `src/picam_qpd_centroid.py`, which:

- captures grayscale camera data,
- center-crops to a square,
- computes quadrant flux sums (`U_A`, `U_B`, `U_C`, `U_D`),
- converts flux imbalance into normalized `(x, y)` error.

So this is a QPD-style analog quadrant metric, not ROI/weighted-centroid pixel location from mapping.

---

## Safety behavior and failure handling

- PID inputs are validated up front (`refresh_ms > 0`, finite gains).
- Commanded voltages are checked against global bounds (`VDIFF_MIN_VOLTS`, `VDIFF_MAX_VOLTS`).
- The FSM layer still enforces hard safety checks on command send.
- On normal stop or on exceptions (`KeyboardInterrupt`, unsafe voltage, hardware failure, etc.), script attempts:
  1. `fsm.set_vdiff(0.0, 0.0)` (best effort),
  2. `fsm.close()`.

This means you should generally end in safe origin unless a lower-level failure prevents it.

---

## Practical tuning notes

- Start with `--mode step` and low `kp`, with `ki=0`, `kd=0`, then add terms slowly.
- If behavior looks unstable/noisy, increase `--refresh-ms` first and tune down gains.
- Be cautious with nonzero `ki`; integral term can accumulate quickly if setpoint mismatch persists.
- Since camera error is normalized quadrant imbalance, gain values are specific to this camera pipeline and are not directly portable to mapping scripts.

---

## Related files worth reading

- `pid_mock_main.py` - main control script and CLI options.
- `src/pid_helpers.py` - PID math, input validation, voltage bound checks.
- `src/picam_qpd_centroid.py` - QPD-style camera error computation and web preview server.
- `src/constants.py` - voltage limits and safety envelope used in command validation.
