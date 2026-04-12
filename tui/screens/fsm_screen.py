from __future__ import annotations

from textual import on
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import Button, Input, Label, RichLog, Switch

from src.fsm_types import FsmConnectResult
from services.fsm_service import FsmService
from tui.widgets import LimitBanner


class FsmPanel(VerticalScroll):
    """Connect / disconnect FSM and apply vdiff (blocking work in workers)."""

    DEFAULT_CSS = """
    FsmPanel RichLog {
        height: 8;
        border: solid $primary;
    }
    FsmPanel #status_line {
        text-style: bold;
    }
    FsmPanel #status_line.armed {
        background: $error;
        color: $text;
    }
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._log: RichLog | None = None

    def compose(self):
        yield LimitBanner()
        yield Label("FSM", classes="section")
        with Horizontal():
            yield Label("I confirm bias/limits — arm hardware")
            yield Switch(id="confirm_arm")
        with Horizontal():
            yield Button("Connect", id="btn_fsm_connect", variant="success")
            yield Button("Disconnect", id="btn_fsm_disconnect", variant="warning")
        yield Label(id="status_line")
        yield Label("Vdiff X / Y (volts)")
        with Horizontal():
            yield Input(placeholder="vdiff_x", id="inp_vx", value="0")
            yield Input(placeholder="vdiff_y", id="inp_vy", value="0")
        yield Button("Apply vdiff", id="btn_apply_vdiff", variant="primary")
        yield RichLog(id="fsm_log", highlight=True, markup=True)

    def on_mount(self) -> None:
        self._log = self.query_one("#fsm_log", RichLog)
        self._refresh_status()

    def _app_fsm(self) -> FsmService:
        return self.app.fsm_service  # type: ignore[attr-defined]

    def _line(self, msg: str) -> None:
        if self._log:
            self._log.write(msg)

    def _refresh_status(self) -> None:
        st = self.query_one("#status_line", Label)
        fsm = self._app_fsm()
        if fsm.is_connected():
            vx, vy = fsm.get_voltages()
            st.update(f"● ARMED   vdiff = ({vx:.3f}, {vy:.3f}) V")
            st.add_class("armed")
        else:
            st.update("○ DISCONNECTED — connect only after physical safety check")
            st.remove_class("armed")

    @on(Button.Pressed, "#btn_fsm_connect")
    def _on_connect(self) -> None:
        if not self.query_one("#confirm_arm", Switch).value:
            self._line("[red]Enable the confirmation switch before connecting.[/]")
            return

        def thread_fn() -> None:
            res = self._app_fsm().connect(confirm=None)
            self.app.call_from_thread(self._after_connect, res)

        self.run_worker(thread_fn, thread=True, exclusive=True, exit_on_error=False)

    def _after_connect(self, res: FsmConnectResult) -> None:
        if res.ok:
            tag = "[yellow]simulated[/]" if res.simulated else "[green]hardware[/]"
            self._line(f"Connected ({tag}): {res.message}")
        else:
            self._line(f"[red]Connect failed:[/] {res.message}")
        self._refresh_status()

    @on(Button.Pressed, "#btn_fsm_disconnect")
    def _on_disconnect(self) -> None:
        def thread_fn() -> None:
            self._app_fsm().disconnect()
            self.app.call_from_thread(self._after_disconnect)

        self.run_worker(thread_fn, thread=True, exclusive=True, exit_on_error=False)

    def _after_disconnect(self) -> None:
        self._line("Disconnected.")
        self._refresh_status()

    @on(Button.Pressed, "#btn_apply_vdiff")
    def _on_apply(self) -> None:
        if not self._app_fsm().is_connected():
            self._line("[red]Not connected.[/]")
            return
        try:
            vx = float(self.query_one("#inp_vx", Input).value or "0")
            vy = float(self.query_one("#inp_vy", Input).value or "0")
        except ValueError:
            self._line("[red]Invalid float for vdiff.[/]")
            return

        def thread_fn() -> None:
            err = None
            out = None
            try:
                out = self._app_fsm().set_vdiff(vx, vy)
            except Exception as e:
                err = e
            self.app.call_from_thread(self._after_apply, out, err)

        self.run_worker(thread_fn, thread=True, exclusive=True, exit_on_error=False)

    def _after_apply(self, out, err) -> None:
        if err is not None:
            self._line(f"[red]{err}[/]")
        elif out is not None:
            self._line(f"Slew result: vdiff = ({out[0]:.4f}, {out[1]:.4f}) V")
        self._refresh_status()
