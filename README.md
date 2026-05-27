# Mirrorctl Raspberry Pi Scripts (`rasppi_src`)

This repository is the Raspberry Pi control layer for FSM voltage actuation, camera-based centroiding, and voltage-to-position mapping workflows.

## Safety-Critical First Notes

- `VDIFF` limits are hardware-critical; stay within `src/constants.py` (`VDIFF_MIN_VOLTS` to `VDIFF_MAX_VOLTS`).
- `VBIAS` and slew settings in `src/constants.py` affect commanded voltages and motion behavior across scripts/services.

## Repository Table Of Contents

Each Top-Level folder contains a `README` with more information about the purpose of that folder as well as how to use the files within. Each `*_main.py` script also has an additional markdown information file that accompanies it. Use this file as the top-level resource to guide you to other files. 

### Top-Level Folders

- `docs/` - documentation for how to use this repository READ THIS
- `src/` - low-level primitives (FSM object/control, voltage helpers, camera capture, centroiding, constants).
- `services/` - orchestration layer (`FsmService`, `MappingService`, `CalibrationService`) used by interactive flows.
- `config/` - camera calibration scripts and docs (`calibrate_camera_interactive.py`, `calibrate_picam.py`, `get_calib_photos.py`, calibration markdown references).
- `tools/` - diagnostics and comparison scripts (centroid/camera/calibration checks).
- `tests/` - pytest coverage for helper and control logic.
- `runs/` - generated CSV outputs and run artifacts (mapping/PID outputs).
- `old/` - legacy scripts/data retained for reference.

### Important Top-Level Files

- `go_to_voltage_main.py` - manual FSM voltage command REPL.
- `pid_mock_main.py` - click-based PID mock runner using centroid error feedback.
- `voltage_mapping_interactive_main.py` - primary interactive or params-driven mapping/verification entrypoint.
- `voltage_mapping_params.json` - parameter template consumed by `voltage_mapping_interactive_main.py` when you choose params-file mode.
- `lab-notebook.md` - experiment history, open issues, and practical observations.

### If You Need X, Go To Y

- "Set or test raw FSM voltages manually" -> `go_to_voltage_main.py` (`docs/go_to_voltage.md`)
- "Run PID loop experiments and log CSV" -> `pid_mock_main.py` (`docs/pid_mock_main.md`)
- "Run mapping sweeps/grids/verify workflows" -> `voltage_mapping_interactive_main.py` (`docs/mapping.md`)
- "Change camera/FSM default behavior globally" -> `src/constants.py` (`docs/constants.md`)
- "Regenerate camera calibration artifacts (`camera_params.npz`)" -> scripts in `config/`
- Troubleshooting -> (`docs/troubleshooting.md`)

## Top-Level `*main.py` Entry Points

### `go_to_voltage_main.py`

Use case:

- Quick manual FSM control loop where you type `vdiffx vdiffy` pairs.
- Best for sanity checks, direct motion probing, and safe manual voltage exploration.

How to run:

```bash
python3 -u go_to_voltage_main.py
```

What happens:

- Prompts interactively for voltage pairs.
- Applies safety checks via `FSM.set_vdiff(...)`; out-of-range requests are rejected.

### `pid_mock_main.py`

Use case:

- Closed-loop PID experiments driven by camera centroid error.
- Optional CSV logging for offline tuning analysis.

How to run (examples):

```bash
python3 -u pid_mock_main.py --refresh-ms 200 --kp 0.01 --ki 0.0 --kd 0.0 --mode auto
python3 -u pid_mock_main.py --refresh-ms 500 --kp 0.02 --ki 0.001 --kd 0.0 --mode step --o runs/pid_tests/
```

What happens:

- Starts FSM and camera server, computes PID deltas, and commands voltage updates each loop.
- In `step` mode, advances on Enter; in `auto`, loops on timer.

### `voltage_mapping_interactive_main.py`

Use case:

- Primary measurement entrypoint for preview/manual/sweep/grid/verify workflows.
- Supports two config styles: interactive prompts or JSON params file.

How to run:

```bash
python3 -u voltage_mapping_interactive_main.py
```

Typical interaction:

- First prompt asks whether to use `voltage_mapping_params.json`.
- If `yes`, it loads config from that JSON.
- If `no`, it asks interactively for mode and relevant fields.

Notes:

- Modes include `preview-cam`, `man`, `sweep`, `sweep-raw`, `random-square-walk`, `grid`, `grid-raw`, `verify`, `verify-sweep`, and `verify-grid`.
- Mapping modes load calibration by default from `config/camera_params.npz` unless `no_calib` is enabled.

## Calibration Settings: Source Of Truth

### Primary Control File: `src/constants.py`

This is the central place for default camera and FSM behavior used across runtime code.

FSM and voltage safety controls:

- `VBIAS`
- `VDIFF_MIN_VOLTS`, `VDIFF_MAX_VOLTS`
- `SLEW_RATE_MS`, `SLEW_AMOUNT_V`
- DAC/SPI/pin constants used during hardware bring-up and command conversion

Camera defaults and controls:

- `CAMERA_DEFAULT_WIDTH`, `CAMERA_DEFAULT_HEIGHT`
- `CAMERA_FRAME_RATE`, `CAMERA_MAIN_FORMAT`
- `CAMERA_AUTO_EXPOSURE_ENABLED`, `CAMERA_EXPOSURE_TIME_US`, `CAMERA_ANALOGUE_GAIN`
- `CAMERA_AUTO_WHITE_BALANCE_ENABLED`, `CAMERA_COLOUR_GAINS`

Centroiding controls:

- `LASER_CENTROID_THRESHOLD_ENABLED`
- `LASER_CENTROID_INTENSITY_THRESHOLD`

How these propagate:

- Camera constants are applied in `src/picam.py` when camera configs are created.
- Voltage/safety constants are enforced through FSM setup/helpers (`src/setup_fsm.py`, `src/voltage_helpers.py`, `src/fsm_obj.py`, service wrappers).
- Mapping and PID scripts consume this behavior via `FSM`, `picam`, and service calls.

## Other Ways Calibration/Settings Change

In addition to `src/constants.py`, runtime behavior can be changed through:

1. `voltage_mapping_params.json`

- Used by `voltage_mapping_interactive_main.py` when params-file mode is selected.
- Controls mode selection, sweep/grid bounds, output location, ROI/resolution, calibration path, and optional Gaussian centroiding inputs.

1. Interactive prompts in `voltage_mapping_interactive_main.py`

- Same effective settings as JSON, but entered during script startup.

1. Calibration artifacts (`config/camera_params.npz`)

- Contains camera calibration data (`mtx`, `dist`, optional `H`).
- Loaded by mapping workflows unless `no_calib` is set.
- Updated/regenerated by scripts under `config/`.

1. Calibration scripts in `config/`

- `calibrate_camera_interactive.py`, `calibrate_picam.py`, and `get_calib_photos.py` can change calibration outputs and therefore mapping interpretation.

Important clarification:

- This repository does not currently use environment-variable overrides for calibration settings; control is through constants, JSON, prompts, and calibration artifacts/scripts.

## Calibration Workflow References (Details)

- `config/CALIBRATION.md` - complete camera calibration and mapping workflow details.
- `config/config.md` - concise calibration quick reference.
- `lab-notebook.md` - practical experiment log, residual observations, and next-step ideas.

## Recommended Handoff Workflow

1. Review `src/constants.py` and confirm safe voltage/camera defaults for the target hardware setup.
2. Confirm/refresh camera calibration artifacts (`config/camera_params.npz`) using scripts in `config/` if setup has changed.
3. Use `voltage_mapping_interactive_main.py` as the primary measurement workflow.
4. Use `go_to_voltage_main.py` for direct manual checks and `pid_mock_main.py` for control experiments.

