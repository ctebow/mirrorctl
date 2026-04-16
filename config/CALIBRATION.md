# Camera calibration and voltage mapping — usage

Run these steps on the **Raspberry Pi** (Picamera2 + OpenCV). Commands assume your **current working directory** is the project root that contains `config/`, `src/`, and `voltage_mapping_main.py` (this repo’s `rasppi_src` folder).

---

## 1. Prerequisites

- Python environment with **`picamera2`**, **`opencv-python`** (or system OpenCV), **`numpy`**, **`click`**.
- ChArUco board that matches **`config/calibrate_picam.py`** (`SQUARES_X`, `SQUARES_Y`, `SQUARE_LENGTH`, `MARKER_LENGTH`, dictionary).
- **Resolution:** Capture, calibration stills, and mapping must use the **same frame size**. Match `src/picam.DEFAULT_FRAME_SIZE` and pass **`--width`** to capture/preview scripts or **`--resolution`** to mapping (height follows `picam` rules for a given width unless you use a full `(w, h)` tuple).

---

## 2. Live preview (headless Pi → Mac browser)

No saving — use this to aim and focus. On the Pi:

```bash
cd /path/to/rasppi_src
python3 config/preview_stream.py --bind 0.0.0.0 --port 8080
```

On your Mac (same LAN): open **`http://<pi-ip>:8080/`** in a browser. Stop with **Ctrl+C** on the Pi.

Optional: `--width 640`, `--fps 24`, `--quality 75`.

---

## 2a. One-command interactive full calibration (recommended)

This runs the whole pipeline in one script:

- starts browser preview (same style as `preview_stream.py`)
- prompts for camera settings (defaults provided)
- lets you capture images with `s/save` and finish with `q/quit`
- runs lens calibration + homography and writes `config/camera_params.npz`
- **always uses legacy CharUco mode** internally

```bash
cd /path/to/rasppi_src
python3 -u config/calibrate_camera_interactive.py
```

Notes:

- Default homography reference is the **first image captured in this session**.
- You can still use `config/calibrate_picam.py` directly for diagnose, custom runs, or homography-only updates.

---

## 3. Capture calibration images (Picamera2, CLI)

Saves JPEGs under `config/calib_images/`. **No GUI** — type commands on the Pi terminal (`s` = save, `q` = quit). Over SSH, use unbuffered Python if keys lag:

```bash
cd /path/to/rasppi_src
python3 -u config/get_calib_photos.py
# optional:  python3 -u config/get_calib_photos.py --width 640
```

Tips:

- Use **10–20** images with the board at different angles and distances.
- Keep lighting and focus representative of mapping runs.
- Each **`s` / `save`** writes a normal **`.jpg`** you can copy off the Pi and open in any image viewer. (`preview_stream.py` does **not** save files; it is preview only.)

---

## 4. Lens calibration (ChArUco → `camera_params.npz`)

Reads `config/calib_images/*.jpg` and writes **`config/camera_params.npz`** with **`mtx`**, **`dist`**, and **`rms`**. By default this step does **not** compute homography **`H`** — you get **undistorted pixel** coordinates in the mapping CSV until you add **`H`** (see §5). Fails if detections are weak or RMS is too high (see `MAX_RMS_PIXELS` in the script).

```bash
cd /path/to/rasppi_src
python3 config/calibrate_picam.py
```

**Board must match the printed CharUco** (grid size, square/marker lengths in the same units, ArUco dictionary). Defaults in the script are a small **5×7** grid; if yours differs, pass e.g. `--squares-x 8 --squares-y 11 --square-length 0.0095 --marker-length 0.007 --dict 0` (see `cv2.aruco` dict constants).

If you get “0 succeeded”:

```bash
python3 config/calibrate_picam.py --diagnose
```

This reports **raw ArUco marker** count, **decoded marker IDs**, and **`CharucoDetector.detectBoard`** corner count on the first image. **0 markers** → wrong `--dict`, blur, or no board in frame. **Markers &gt; 0 but CharUco corner count `None`/0** → often **legacy vs new CharUco layout** (try **`--legacy-charuco`** with the same other flags), or wrong **`--squares-x` / `--squares-y`**, or **`--dict`** (e.g. `DICT_4X4_50` only allows marker ids **0–49**). **Square/marker lengths** can be in mm or m as long as the **ratio** matches the printed board (e.g. 30 and 23 is the same ratio as 0.03 and 0.023).

**Legacy CharUco (typical 8×6 print, DICT_4X4_50)** — exact command used when the default OpenCV 4.7+ CharUco layout does not match the PDF:

```bash
cd /path/to/rasppi_src
python3 config/calibrate_picam.py --legacy-charuco --squares-x 8 --squares-y 6 --square-length 30 --marker-length 23 --dict 0
```

Same flags with **`--diagnose`** (first image only, no `npz` write):

```bash
python3 config/calibrate_picam.py --legacy-charuco --squares-x 8 --squares-y 6 --square-length 30 --marker-length 23 --dict 0 --diagnose
```

---

## 4a. What actually feeds centroiding / voltage mapping

This is **not** automatic from capture alone:

| Artifact | Used when mapping runs? |
|----------|-------------------------|
| `config/calib_images/*.jpg` | **No** — offline input for `calibrate_picam.py` only. |
| `config/camera_params.npz` | **Yes** (default) — `voltage_mapping_main.py` loads **`mtx` / `dist` / optional `H`** from this file. |

- After you capture **new** JPEGs, you must **run `calibrate_picam.py` again** (or use **`--update-homography`** if you only change **`H`**) so **`camera_params.npz`** matches. Mapping does **not** re-read the JPEG folder at runtime.
- If **`npz` is missing** and you did not pass **`--no-calib`**, mapping will stop and tell you to run calibration first.

---

## 5. Homography (optional — mm coordinates in CSV)

**Homography is optional.** If you skip this section entirely, mapping still applies **lens undistortion** using **`mtx` / `dist`** and writes **`cx_ud_px`**, **`cy_ud_py`** (undistorted pixels). You only need **`H`** if you want **board-plane millimetres** in the CSV (**`x_mm`**, **`y_mm`**).

**`H` is not a separate parameter file.** It is stored in the same `config/camera_params.npz` as `mtx` and `dist`, alongside optional `rms`. You add it by passing **one** reference image (board clearly visible) on the command line — there is no extra standalone homography config.

Homography **`H`** maps undistorted **pixels** to the **ChArUco board plane** (units follow your **`SQUARE_LENGTH`** etc. in `calibrate_picam.py`). Use an image where the **full board is clearly visible** and the board is in the **same physical relationship** to the camera you care about during laser mapping.

### Option A — lens cal + homography in one run

Runs lens calibration from `calib_images`, then computes **`H`** from the reference image and saves everything into one `npz`:

```bash
cd /path/to/rasppi_src
python3 config/calibrate_picam.py --homography-ref /path/to/board_visible.jpg
```

### Option B — update **`H`** only (keep existing `mtx` / `dist`)

After you move the board or camera mount but **not** lens focus or resolution:

```bash
cd /path/to/rasppi_src
python3 config/calibrate_picam.py --update-homography /path/to/board_visible.jpg
```

---

## 6. Voltage mapping (uses calibration by default)

`voltage_mapping_main.py` loads **`config/camera_params.npz`** automatically, applies **undistortion**, then optionally **mm** via **`H`** if present.

### Default calibration path + resolution

```bash
cd /path/to/rasppi_src
python3 voltage_mapping_main.py --resolution 640 --mode man
```

### Custom `npz` location

```bash
python3 voltage_mapping_main.py --calibration /path/to/camera_params.npz --mode auto --axis x --start 0 --end 175 --step-size 1
```

### Raw pixels (no `npz`; for quick tests)

```bash
python3 voltage_mapping_main.py --no-calib --mode man
```

### Camera sanity check (writes `gray1.jpg`; undistorted if `npz` exists)

```bash
python3 voltage_mapping_main.py --mode test-cam
```

---

## 7. CSV output columns

The first two columns are always **`vdiffx`**, **`vdiffy`**. The centroid columns depend on calibration:

| Situation | Columns 3–4 |
|-----------|----------------|
| Default: `npz` with `mtx`, `dist` only | `cx_ud_px`, `cy_ud_px` (undistorted pixels) |
| `npz` also contains **`H`** | `x_mm`, `y_mm` (board plane) |
| **`--no-calib`** | `cx_raw`, `cy_raw` |

---

## 8. When to redo which step

| Change | Redo capture | Redo `calibrate_picam.py` (lens) | Redo **`H`** |
|--------|----------------|----------------------------------|--------------|
| Lens focus, zoom, or **resolution** | Yes | Yes | Yes |
| Camera rigid move, **same** focus/resolution | Optional for lens | Usually no | **Yes** if you use mm |
| Board moved / scene geometry changed | No (for lens) | No | **Yes** |
| New printed board or wrong board params in script | Yes | Yes | Yes (after new lens cal) |

---

## 9. File locations

| File / directory | Role |
|------------------|------|
| `config/preview_stream.py` | MJPEG server for live browser preview |
| `config/calibrate_camera_interactive.py` | End-to-end interactive flow: preview + capture + lens calibration + homography (legacy CharUco forced). |
| `config/calib_images/` | Viewable JPEGs from `get_calib_photos.py` — input to `calibrate_picam.py` only; **not** read by mapping at runtime. |
| `config/camera_params.npz` | Output of `calibrate_picam.py`: `mtx`, `dist`, `rms`, optional `H` — **this** is what `voltage_mapping_main.py` loads (unless `--no-calib`). |
| `voltage_mapping_out.csv` (or `-o`) | Mapping output |
