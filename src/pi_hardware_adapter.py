"""
Low-level Raspberry Pi SPI / pigpio lifecycle. Keeps ``setup_fsm`` as the source of truth
for the order-sensitive sequence; this adapter is the injectable boundary for tests/TUI.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Optional, Tuple

from . import setup_fsm


class PiHardwareAdapter:
    """Thin wrapper around ``setup_fsm.fsm_begin`` / ``fsm_close`` for dependency injection."""

    def begin(
        self,
        *,
        confirm: Optional[Callable[[str], bool]] = None,
        confirm_message: Optional[str] = None,
    ) -> Tuple[Any, Any]:
        return setup_fsm.fsm_begin(confirm=confirm, confirm_message=confirm_message)

    def begin_interactive(self) -> Tuple[Any, Any]:
        return setup_fsm.fsm_begin_interactive()

    def close(self, start_state: tuple, slew_params: tuple, spi: Any, pi: Any) -> None:
        setup_fsm.fsm_close(start_state, slew_params, spi, pi)
