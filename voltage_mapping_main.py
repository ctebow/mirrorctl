"""
Voltage-to-centroid mapping: thin Click CLI over ``MappingService`` / ``CalibrationService``.
"""
from __future__ import annotations

from pathlib import Path

import click

from src import FSM, centroiding, picam
from src.exceptions import CalibrationError, HardwareUnavailable, UnsafeVoltageRequest
from services import CalibrationService, MappingService, MappingSweepParams

_DEFAULT_CAL = Path(__file__).resolve().parent / "config" / "camera_params.npz"
_DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "runs" / "mapping"
mapper = MappingService()


def _open_camera(resolution: int):
    try:
        return picam.init_camera(resolution)
    except Exception as exc:
        raise HardwareUnavailable(f"Camera failed: {exc}") from exc


@click.command()
@click.option("-n", "--num_frames", default=5, type=int, help="Frames per voltage step")
@click.option("-t", "--settling_time", default=0.1, type=float, help="Seconds between steps (auto)")
@click.option("-a", "--axis", default="x", type=str, help="Sweep axis: x or y (auto mode)")
@click.option(
    "-o",
    "--outfile",
    default=str(_DEFAULT_OUT_DIR / "voltage_mapping_out.csv"),
    type=str,
    help="CSV output path (default: runs/mapping/voltage_mapping_out.csv)",
)
@click.option("-s", "--step-size", default=1.0, type=float, help="VDIFF step (V)")
@click.option("--start", default=0.0, type=float, help="Start voltage (V)")
@click.option("--end", default=175.0, type=float, help="End voltage (V)")
@click.option("--start-x", default=0.0, type=float, help="Grid start vdiff_x (V)")
@click.option("--start-y", default=0.0, type=float, help="Grid start vdiff_y (V)")
@click.option("--end-x", default=0.0, type=float, help="Grid end vdiff_x (V)")
@click.option("--end-y", default=0.0, type=float, help="Grid end vdiff_y (V)")
@click.option("--step-x", default=0.0, type=float, help="Grid step on x axis (V); 0 keeps x fixed")
@click.option("--step-y", default=0.0, type=float, help="Grid step on y axis (V); 0 keeps y fixed")
@click.option("--resolution", default=640, type=int, help="Frame width (height 480)")
@click.option("--roi", default=50, type=int, help="Centroid ROI half-size (px)")
@click.option("--mode", default="man", type=str, help="auto | grid | man | test-cam")
@click.option("--image_outfile", default=None, type=str, help="If set, write one grayscale frame (test-cam)")
@click.option(
    "--calibration",
    "calibration_path",
    type=click.Path(path_type=Path, dir_okay=False),
    default=_DEFAULT_CAL,
    help="camera_params.npz",
)
@click.option("--no-calib", is_flag=True, help="Raw sensor pixels only")
def cmd(
    num_frames,
    settling_time,
    axis,
    outfile,
    step_size,
    start,
    end,
    start_x,
    start_y,
    end_x,
    end_y,
    step_x,
    step_y,
    resolution,
    roi,
    mode,
    image_outfile,
    calibration_path,
    no_calib,
):
    calib: centroiding.CameraCalibration | None = None
    cam = None
    fsm = FSM()

    try:
        if mode == "test-cam":
            cam = _open_camera(resolution)
            if not no_calib:
                try:
                    calib = CalibrationService.load(calibration_path)
                except CalibrationError as exc:
                    click.echo(str(exc), err=True)
                    return 1
            gray = picam.get_gray_frame(cam)
            if calib is not None:
                gray = calib.undistort_gray(gray)
            out = image_outfile or "gray1.jpg"
            centroiding.grayscale_to_outfile(gray, out)
            click.echo(f"Wrote {out}")
            return 0

        cam = _open_camera(resolution)

        if not no_calib:
            try:
                calib = CalibrationService.load(calibration_path)
            except CalibrationError as exc:
                click.echo(str(exc), err=True)
                click.echo("Use --no-calib or run config/calibrate_picam.py first.", err=True)
                return 1

        header = mapper.csv_header(calib)

        result = fsm.begin_interactive()
        if not result.ok:
            click.echo("FSM failed to start or was aborted.", err=True)
            return 1

        coords: list[list] = []

        if mode == "man":
            click.echo(f"[MAN] FSM vdiff: {fsm.vdiff_x} | {fsm.vdiff_y}")
            try:
                while True:
                    usr = input("Input vdiffx, vdiffy (x y): ")
                    lst = usr.split()
                    if len(lst) != 2:
                        click.echo("Bad coords. Input <x y>")
                        continue
                    x, y = lst[0], lst[1]
                    try:
                        fx, fy = float(x), float(y)
                    except ValueError:
                        click.echo("Bad coords. Input <x y>")
                        continue
                    try:
                        fsm.set_vdiff(fx, fy)
                    except UnsafeVoltageRequest as exc:
                        click.echo(f"Unsafe: {exc}", err=True)
                        continue
                    centroid = mapper.capture_centroid_averages(cam, num_frames, roi, calib)
                    vx, vy = fsm.get_voltages()
                    coords.append([vx, vy, *centroid])
            except KeyboardInterrupt:
                click.echo("Interrupted — saving CSV")
            finally:
                fsm.close()
                Path(outfile).parent.mkdir(parents=True, exist_ok=True)
                mapper.write_csv(Path(outfile), coords, header, mode="w")
            return 0

        if mode == "auto":
            if axis not in ("x", "y"):
                click.echo("[AUTO] axis must be x or y", err=True)
                fsm.close()
                return 1
            params = MappingSweepParams(
                num_frames=num_frames,
                settling_time=settling_time,
                axis=axis,
                step_size=step_size,
                start=start,
                end=end,
                roi=roi,
            )
            rows: list[list] = []
            try:
                rows = mapper.run_auto_sweep(fsm, cam, params, calib)
            except (ValueError, UnsafeVoltageRequest) as exc:
                click.echo(str(exc), err=True)
                fsm.close()
                return 1
            fsm.close()
            Path(outfile).parent.mkdir(parents=True, exist_ok=True)
            mapper.write_csv(Path(outfile), rows, header, mode="w")
            click.echo(f"Wrote {outfile} ({len(rows)} rows)")
            return 0

        if mode == "grid":
            params = MappingSweepParams(
                num_frames=num_frames,
                settling_time=settling_time,
                axis=axis,
                step_size=step_size,
                start=start,
                end=end,
                roi=roi,
                start_x=start_x,
                start_y=start_y,
                end_x=end_x,
                end_y=end_y,
                step_x=step_x,
                step_y=step_y,
            )
            rows: list[list] = []
            try:
                rows = mapper.run_grid_sweep(fsm, cam, params, calib)
            except (ValueError, UnsafeVoltageRequest) as exc:
                click.echo(str(exc), err=True)
                fsm.close()
                return 1
            fsm.close()
            Path(outfile).parent.mkdir(parents=True, exist_ok=True)
            grid_header = ["sweep_dir", *header]
            mapper.write_csv(Path(outfile), rows, grid_header, mode="w")
            click.echo(f"Wrote {outfile} ({len(rows)} rows)")
            return 0

        click.echo("Invalid mode", err=True)
        fsm.close()
        return 1

    except HardwareUnavailable as exc:
        click.echo(str(exc), err=True)
        return 1
    finally:
        picam.close_camera(cam)


if __name__ == "__main__":
    cmd()
