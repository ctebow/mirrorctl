from __future__ import annotations

import argparse

from textual.app import App, ComposeResult
from textual.containers import Vertical
from textual.widgets import Footer, Header, Static, TabbedContent, TabPane

from services.fsm_service import FsmService
from tui.screens.diagnostic_screen import DiagnosticPanel
from tui.screens.fsm_screen import FsmPanel
from tui.screens.mapping_screen import MappingPanel


class MirrorctlApp(App[None]):
    """Operator TUI for FSM, mapping sweep, and centroid checks."""

    TITLE = "mirrorctl"
    CSS = """
    TabbedContent { height: 1fr; }
    """
    BINDINGS = [
        ("q", "quit", "Quit"),
        ("d", "toggle_dark", "Dark"),
    ]

    def __init__(self, *, dry_run: bool = False):
        super().__init__()
        self.dry_run = dry_run
        self.fsm_service = FsmService(dry_run=dry_run)

    def compose(self) -> ComposeResult:
        yield Header()
        if self.dry_run:
            yield Static(
                "[yellow]DRY-RUN:[/] Simulated FSM — no SPI; vision tabs still need a Pi camera.",
                id="dry_banner",
            )
        with Vertical():
            with TabbedContent(id="tabs"):
                with TabPane("FSM", id="tab_fsm"):
                    yield FsmPanel()
                with TabPane("Mapping", id="tab_map"):
                    yield MappingPanel()
                with TabPane("Diagnose", id="tab_diag"):
                    yield DiagnosticPanel()
        yield Footer()

    def action_toggle_dark(self) -> None:
        self.theme = "textual-dark" if self.theme != "textual-dark" else "textual-light"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="mirrorctl Textual operator UI")
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Simulated FSM only (no SPI); camera still required for vision tabs on device.",
    )
    return p.parse_args(argv)
