from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FsmConnectResult:
    """Result of FSM.begin() / FsmService.connect()."""

    ok: bool
    simulated: bool
    message: str = ""
