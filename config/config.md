# Configuration quick reference

## ChArUco lens calibration — legacy CharUco pattern

Many printed / PDF CharUco targets match OpenCV’s **legacy** board layout. If markers decode but calibration reports **0** accepted views, use **`--legacy-charuco`** and board size / lengths that match the print (units only need a consistent ratio).

From the project root (`rasppi_src`), after JPEGs are in `config/calib_images/`:

```bash
cd /path/to/rasppi_src
python3 config/calibrate_picam.py --legacy-charuco --squares-x 8 --squares-y 6 --square-length 30 --marker-length 23 --dict 0
```

Optional: append **`--diagnose`** to inspect ArUco / CharUco detection on the **first** image only (no full calibration).

Full workflow, homography, and mapping: **`config/CALIBRATION.md`**.
