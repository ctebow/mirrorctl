from __future__ import annotations

import logging
from collections.abc import Callable
from typing import TYPE_CHECKING, Optional, Tuple

from src import constants
from src.exceptions import UnsafeVoltageRequest
from src.fsm_types import FsmConnectResult

if TYPE_CHECKING:
    from src.fsm_obj import FSM

log = logging.getLogger(__name__)


class FsmService:
    """
    Application-facing FSM API: wraps ``FSM`` with logging and optional dry-run stub.
    """

    def __init__(self, fsm: Optional["FSM"] = None, *, dry_run: bool = False):
        self._dry_run = dry_run
        if dry_run:
            self._fsm = _DryRunFsm()
        else:
            from src.fsm_obj import FSM as _FSM

            self._fsm = fsm or _FSM()

    @property
    def inner(self) -> "FSM | _DryRunFsm":
        return self._fsm

    def connect(
        self,
        *,
        confirm: Optional[Callable[[str], bool]] = None,
        interactive: bool = False,
    ) -> FsmConnectResult:
        if interactive:
            return self._fsm.begin_interactive()
        return self._fsm.begin(confirm=confirm)

    def disconnect(self) -> None:
        self._fsm.close()

    def is_connected(self) -> bool:
        return self._fsm.is_active()

    def get_voltages(self) -> Tuple[float, float]:
        return self._fsm.get_voltages()

    def set_vdiff(self, vx: float, vy: float) -> Tuple[float, float]:
        try:
            return self._fsm.set_vdiff(vx, vy)
        except UnsafeVoltageRequest:
            raise

    def slew_stats(self) -> tuple:
        return self._fsm.get_slew_stats()


class _DryRunFsm:
    """In-memory stand-in for UI development without hardware."""

    def __init__(self):
        self.vdiff_x = 0.0
        self.vdiff_y = 0.0
        self.spi = None
        self.enable = None
        self.slew_time = 0.0001
        self.slew_step = 0.25

    def begin(self, *, confirm=None) -> FsmConnectResult:
        self.vdiff_x = 0.0
        self.vdiff_y = 0.0
        self.spi = "DRY_RUN"
        self.enable = "DRY_RUN"
        return FsmConnectResult(ok=True, simulated=True, message="Dry-run FSM (no hardware)")

    def begin_interactive(self) -> FsmConnectResult:
        return self.begin(confirm=None)

    def close(self) -> None:
        self.spi = None
        self.enable = None
        self.vdiff_x = 0.0
        self.vdiff_y = 0.0

    def is_active(self) -> bool:
        return self.spi is not None

    def get_slew_stats(self) -> tuple:
        return (self.slew_time, self.slew_step)

    def get_voltages(self) -> tuple:
        return (self.vdiff_x, self.vdiff_y)

    def set_vdiff(self, vx: float, vy: float) -> Tuple[float, float]:
        lo, hi = constants.VDIFF_MIN_VOLTS, constants.VDIFF_MAX_VOLTS
        if not (lo <= vx <= hi):
            raise UnsafeVoltageRequest(f"vdiff_x={vx} not in [{lo}, {hi}]", axis="x")
        if not (lo <= vy <= hi):
            raise UnsafeVoltageRequest(f"vdiff_y={vy} not in [{lo}, {hi}]", axis="y")
        self.vdiff_x = float(vx)
        self.vdiff_y = float(vy)
        return (self.vdiff_x, self.vdiff_y)

    def update_slew(self, st, ss) -> int:
        self.slew_time = st
        self.slew_step = ss
        return 0
