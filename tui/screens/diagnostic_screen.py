from __future__ import annotations

from pathlib import Path

from textual import on
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import Button, Input, Label, RichLog

from services import CalibrationService, MappingService
from tui.widgets import LimitBanner

_REPO = Path(__file__).resolve().parents[2]


class DiagnosticPanel(VerticalScroll):
    """Single centroid sample (camera + calibration)."""

    DEFAULT_CSS = """
    DiagnosticPanel RichLog {
        height: 10;
        border: solid $primary;
    }
    """

    def compose(self):
        yield LimitBanner()
        yield Label("Centroid diagnostic", classes="section")
        with Horizontal():
            yield Input(placeholder="frames", id="diag_frames", value="3")
            yield Input(placeholder="roi", id="diag_roi", value="50")
            yield Input(placeholder="width", id="diag_res", value="640")
        yield Input(
            placeholder="calibration npz",
            id="diag_cal",
            value=str(_REPO / "config" / "camera_params.npz"),
        )
        yield Button("Capture centroid", id="btn_centroid", variant="primary")
        yield RichLog(id="diag_log", highlight=True, markup=True)

    def on_mount(self) -> None:
        self._log = self.query_one("#diag_log", RichLog)

    def _line(self, msg: str) -> None:
        self._log.write(msg)

    @on(Button.Pressed, "#btn_centroid")
    def _on_capture(self) -> None:
        app = self.app  # type: ignore[assignment]

        try:
            nfr = int(self.query_one("#diag_frames", Input).value)
            roi = int(self.query_one("#diag_roi", Input).value)
            res = int(self.query_one("#diag_res", Input).value)
        except ValueError:
            self._line("[red]Invalid integers.[/]")
            return
        cal_path = Path(self.query_one("#diag_cal", Input).value)

        def thread_fn() -> None:
            from src import picam

            lines: list[str] = []
            cam = None
            try:
                calib = CalibrationService.load(cal_path)
            except Exception as e:
                lines.append(f"[red]{e}[/]")
                app.call_from_thread(self._flush_log, lines)
                return
            try:
                cam = picam.init_camera(res)
            except Exception as e:
                lines.append(f"[red]Camera: {e}[/]")
                app.call_from_thread(self._flush_log, lines)
                return
            try:
                mapper = MappingService()
                row = mapper.capture_centroid_averages(cam, nfr, roi, calib)
                lines.append(f"Centroid row: {row}")
            except Exception as e:
                lines.append(f"[red]{e}[/]")
            finally:
                picam.close_camera(cam)
            app.call_from_thread(self._flush_log, lines)

        self.run_worker(thread_fn, thread=True, exclusive=True, exit_on_error=False)

    def _flush_log(self, lines: list[str]) -> None:
        for ln in lines:
            self._line(ln)
