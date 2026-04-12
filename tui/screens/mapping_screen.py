from __future__ import annotations

from pathlib import Path

from textual import on
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import Button, Input, Label, RichLog

from services import CalibrationService, MappingService, MappingSweepParams
from tui.widgets import LimitBanner

_REPO = Path(__file__).resolve().parents[2]


class MappingPanel(VerticalScroll):
    """Axis sweep: connect FSM on the FSM tab first (unless dry-run)."""

    DEFAULT_CSS = """
    MappingPanel RichLog {
        height: 10;
        border: solid $primary;
    }
    """

    def compose(self):
        yield LimitBanner()
        yield Label("Voltage mapping sweep", classes="section")
        yield Label("Requires FSM connected (FSM tab) and camera on device.")
        with Horizontal():
            yield Input(placeholder="axis x|y", id="inp_axis", value="x")
            yield Input(placeholder="start V", id="inp_start", value="0")
            yield Input(placeholder="end V", id="inp_end", value="175")
            yield Input(placeholder="step", id="inp_step", value="1")
        with Horizontal():
            yield Input(placeholder="frames", id="inp_frames", value="5")
            yield Input(placeholder="roi", id="inp_roi", value="50")
            yield Input(placeholder="settle s", id="inp_settle", value="0.1")
        yield Input(
            placeholder="CSV path",
            id="inp_out",
            value=str(_REPO / "runs" / "mapping" / "tui_sweep.csv"),
        )
        yield Input(
            placeholder="calibration npz",
            id="inp_cal",
            value=str(_REPO / "config" / "camera_params.npz"),
        )
        yield Button("Run auto sweep", id="btn_sweep", variant="primary")
        yield RichLog(id="map_log", highlight=True, markup=True)

    def on_mount(self) -> None:
        self._log = self.query_one("#map_log", RichLog)

    def _line(self, msg: str) -> None:
        self._log.write(msg)

    @on(Button.Pressed, "#btn_sweep")
    def _on_sweep(self) -> None:
        app = self.app  # type: ignore[assignment]
        if not app.fsm_service.is_connected():  # type: ignore[attr-defined]
            self._line("[red]Connect FSM on the FSM tab first.[/]")
            return

        try:
            start = float(self.query_one("#inp_start", Input).value)
            end = float(self.query_one("#inp_end", Input).value)
            step = float(self.query_one("#inp_step", Input).value)
            nfr = int(self.query_one("#inp_frames", Input).value)
            roi = int(self.query_one("#inp_roi", Input).value)
            settle = float(self.query_one("#inp_settle", Input).value)
        except ValueError:
            self._line("[red]Invalid numeric parameters.[/]")
            return

        axis = (self.query_one("#inp_axis", Input).value or "x").strip().lower()
        if axis not in ("x", "y"):
            self._line("[red]Axis must be x or y.[/]")
            return

        out_path = Path(self.query_one("#inp_out", Input).value)
        cal_path = Path(self.query_one("#inp_cal", Input).value)

        params = MappingSweepParams(
            num_frames=nfr,
            settling_time=settle,
            axis=axis,
            step_size=step,
            start=start,
            end=end,
            roi=roi,
        )

        def thread_fn() -> None:
            from src import picam

            mapper = MappingService()
            log_lines: list[str] = []
            rows_out: list | None = None
            cam = None
            try:
                cam = picam.init_camera(640)
            except Exception as e:
                log_lines.append(f"[red]Camera: {e}[/]")
                app.call_from_thread(self._flush_log, log_lines)
                return

            try:
                calib = CalibrationService.load(cal_path)
            except Exception as e:
                picam.close_camera(cam)
                log_lines.append(f"[red]Calibration: {e}[/]")
                app.call_from_thread(self._flush_log, log_lines)
                return

            fsm = app.fsm_service.inner  # type: ignore[attr-defined]
            try:

                def prog(_i: int, v: float, row: list) -> None:
                    log_lines.append(f"vdiff≈{v:.2f} → {row!r}")

                rows_out = mapper.run_auto_sweep(fsm, cam, params, calib, progress=prog)
                header = mapper.csv_header(calib)
                out_path.parent.mkdir(parents=True, exist_ok=True)
                mapper.write_csv(out_path, rows_out, header, mode="w")
                log_lines.append(f"[green]Wrote {out_path} ({len(rows_out)} rows)[/]")
            except Exception as e:
                log_lines.append(f"[red]{e}[/]")
            finally:
                picam.close_camera(cam)
            app.call_from_thread(self._flush_log, log_lines)

        self.run_worker(thread_fn, thread=True, exclusive=True, exit_on_error=False)

    def _flush_log(self, log_lines: list[str]) -> None:
        for ln in log_lines:
            self._line(ln)
