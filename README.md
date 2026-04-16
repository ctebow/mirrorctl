# Mirrorctl Raspberry Pi Scripts (`rasppi_src`)

Script-first control and calibration tools for a Mirrorcle-style 2-axis MEMS mirror on Raspberry Pi.

This repo centers on:
- safe FSM voltage control over SPI/GPIO
- camera-assisted voltage-to-position mapping
- ChArUco-based camera calibration for undistortion and optional board-plane millimeter coordinates

---

## Safety First

- Differential voltage (`VDIFF`) limits are hardware-critical. Keep values within limits defined in `src/constants.py`.
- `FSM.close()` slews back toward neutral before disabling the driver; always call it in `finally`.
- Software checks are guardrails, not a substitute for hardware review and wiring validation.

---

## Quick Start

From the `rasppi_src` root:

1. Calibrate camera (recommended one-command flow):
   - `python3 -u config/calibrate_camera_interactive.py`
2. Run voltage mapping:
   - `python3 voltage_mapping_main.py --resolution 640 --mode man`
3. Manual voltage control (no camera):
   - `python3 go_to_voltage_main.py`

For full camera calibration details and troubleshooting, see `config/CALIBRATION.md`.

---

## Repository Layout

- `src/` — core hardware and vision primitives
  - `fsm_obj.py`: `FSM` class for connect/set/close operations
  - `setup_fsm.py`: ordered hardware bring-up / shutdown sequence
  - `voltage_helpers.py`: DAC conversion and slew helpers
  - `picam.py`: Picamera2 capture helpers
  - `centroiding.py`: laser centroid + calibration model loader
  - `constants.py`: pin mappings, voltage/slew limits, SPI settings
- `services/` — application-layer orchestration
  - `FsmService`, `MappingService`, `CalibrationService`
- `config/` — camera workflow scripts/docs
  - `calibrate_camera_interactive.py` (preview + capture + lens + homography)
  - `get_calib_photos.py` (capture-only)
  - `calibrate_picam.py` (lens/homography CLI utility, supports update-only homography)
  - `preview_stream.py` (headless browser stream)
  - `CALIBRATION.md` (canonical calibration workflow)
- `tools/` — diagnostics utilities (centroid and distance helpers)
- `go_to_voltage_main.py`, `voltage_mapping_main.py` — main user-facing scripts

---

## Core API (`src/FSM`)

Import:

```python
from src import FSM
from src.exceptions import UnsafeVoltageRequest
```

### `begin()` and connection model

- `FSM.begin(confirm=None) -> FsmConnectResult`
  - Returns `ok`, `simulated`, and `message`
  - On Linux, attempts real hardware init through `setup_fsm`
  - On non-Linux, returns simulated mode
- `FSM.begin_interactive() -> FsmConnectResult`
  - Uses interactive confirmation path

### Set voltages safely

- `FSM.set_vdiff(vx, vy) -> tuple[float, float]`
- Raises `UnsafeVoltageRequest` if requested voltage is out of allowed range

### Shutdown

- `FSM.close()` disconnects and returns to safe neutral state path

---

## Script Entry Points

- `go_to_voltage_main.py`
  - interactive voltage REPL (`vdiffx vdiffy`) with range enforcement
- `voltage_mapping_main.py`
  - camera-based mapping (`man`, `auto`, `test-cam`)
  - optional calibration file via `--calibration`
  - raw mode with `--no-calib`
- `config/calibrate_camera_interactive.py`
  - starts preview stream, captures with `s/save`, calibrates lens + homography, writes `config/camera_params.npz`
  - legacy CharUco mode forced on in this workflow
- `config/calibrate_picam.py`
  - standalone calibration utility, including `--update-homography`

---

## Camera Calibration + Mapping Data Flow

1. Capture ChArUco images.
2. Solve lens intrinsics (`mtx`, `dist`) and reprojection quality (`rms`).
3. Optionally compute homography (`H`) to board plane.
4. Save `config/camera_params.npz`.
5. `voltage_mapping_main.py` loads this file by default and outputs:
   - undistorted pixels (`cx_ud_px`, `cy_ud_px`) when only `mtx`/`dist` are present
   - board-plane mm (`x_mm`, `y_mm`) when `H` is present

---

## Error Model

Key domain exceptions in `src/exceptions.py`:

- `UnsafeVoltageRequest` — requested voltage outside configured range
- `HardwareUnavailable` — camera/SPI/GPIO setup failure
- `CalibrationError` — missing/invalid calibration artifacts
- `InitializationAborted` — setup refused by confirmation/policy

Scripts catch and report these in user-facing flows.

---

## Configuration Source Of Truth

`src/constants.py` defines:
- voltage and slew limits
- SPI settings
- enable pin and PWM/FCLK settings

Any hardware setup changes should be made there and validated against your schematic.

---

## Related Docs

- `config/CALIBRATION.md` — full calibration workflow, homography details, troubleshooting
- `config.md` — concise calibration quick reference
