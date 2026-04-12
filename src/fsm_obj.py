"""
FSM class for python testing dev.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Optional, Tuple

from . import setup_fsm
from . import constants
from . import voltage_helpers as helpers
from .exceptions import UnsafeVoltageRequest
from .fsm_types import FsmConnectResult

log = logging.getLogger(__name__)

V_MAX = constants.VDIFF_MAX_VOLTS


class FSM:
    def __init__(self, slew_time=None, slew_step=None):
        self.vdiff_x = 0
        self.vdiff_y = 0
        self.spi = None
        self.enable = None

        if slew_time:
            self.slew_time = slew_time
        else:
            self.slew_time = setup_fsm.SLEW_RATE_MS
        if slew_step:
            self.slew_step = slew_step
        else:
            self.slew_step = setup_fsm.SLEW_AMOUNT_V

    def begin(
        self,
        *,
        confirm: Optional[Callable[[str], bool]] = None,
    ) -> FsmConnectResult:
        """
        Begin FSM. On Linux, runs hardware setup via ``setup_fsm.fsm_begin``.
        If ``confirm`` is None, no prompt is shown (caller must have obtained operator approval).
        Use ``begin_interactive()`` for stdin ``Y`` confirmation (legacy CLI).
        """
        self.vdiff_x = 0.0
        self.vdiff_y = 0.0
        if setup_fsm.IS_LINUX:
            log.info("Linux detected — initializing FSM hardware")
            self.spi, self.enable = setup_fsm.fsm_begin(confirm=confirm)
            if self.spi is not None and self.enable is not None:
                return FsmConnectResult(ok=True, simulated=False, message="Hardware connected")
            return FsmConnectResult(ok=False, simulated=False, message="Initialization failed or aborted")
        self.spi = "TEST"
        self.enable = "TEST"
        return FsmConnectResult(ok=True, simulated=True, message="Simulated FSM (not on Linux / no drivers)")

    def begin_interactive(self) -> FsmConnectResult:
        """Stdin confirmation (``Y``) then hardware setup on Linux."""
        self.vdiff_x = 0.0
        self.vdiff_y = 0.0
        if setup_fsm.IS_LINUX:
            self.spi, self.enable = setup_fsm.fsm_begin_interactive()
            if self.spi is not None and self.enable is not None:
                return FsmConnectResult(ok=True, simulated=False, message="Hardware connected")
            return FsmConnectResult(ok=False, simulated=False, message="Initialization failed or aborted")
        self.spi = "TEST"
        self.enable = "TEST"
        return FsmConnectResult(ok=True, simulated=True, message="Simulated FSM (not on Linux / no drivers)")

    def close(self) -> None:
        if not self.spi:
            log.info("FSM close: not active")
            return

        setup_fsm.fsm_close(
            (self.vdiff_x, self.vdiff_y),
            (self.slew_time, self.slew_step),
            self.spi,
            self.enable,
        )
        self.spi = None
        self.enable = None
        self.vdiff_x = 0
        self.vdiff_y = 0

    def is_active(self) -> bool:
        return bool(self.spi and self.enable)

    def get_slew_stats(self) -> tuple:
        log.info("Slew rate: %s, Slew step: %s", self.slew_time, self.slew_step)
        return (self.slew_time, self.slew_step)

    def get_voltages(self) -> tuple:
        return (self.vdiff_x, self.vdiff_y)

    def update_slew(self, slew_time, slew_step) -> int:
        self.slew_time = slew_time
        self.slew_step = slew_step
        return 0

    def _check_vdiff_range(self, vdiff_x_new: float, vdiff_y_new: float) -> None:
        lo, hi = setup_fsm.VDIFF_MIN_VOLTS, setup_fsm.VDIFF_MAX_VOLTS
        if not (lo <= vdiff_x_new <= hi):
            raise UnsafeVoltageRequest(
                f"vdiff_x={vdiff_x_new} not in [{lo}, {hi}]",
                axis="x",
            )
        if not (lo <= vdiff_y_new <= hi):
            raise UnsafeVoltageRequest(
                f"vdiff_y={vdiff_y_new} not in [{lo}, {hi}]",
                axis="y",
            )

    def set_vdiff(self, vdiff_x_new: float, vdiff_y_new: float) -> Tuple[float, float]:
        self._check_vdiff_range(vdiff_x_new, vdiff_y_new)

        res = helpers.slew(
            (self.vdiff_x, self.vdiff_y),
            (vdiff_x_new, vdiff_y_new),
            (self.slew_time, self.slew_step),
            self.spi,
        )

        if res[0] != vdiff_x_new:
            log.warning("Slew may not have reached requested vdiff_x (got %s, wanted %s)", res[0], vdiff_x_new)
        if res[1] != vdiff_y_new:
            log.warning("Slew may not have reached requested vdiff_y (got %s, wanted %s)", res[1], vdiff_y_new)

        self.vdiff_x = res[0]
        self.vdiff_y = res[1]

        return res
