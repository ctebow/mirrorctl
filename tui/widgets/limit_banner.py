from __future__ import annotations

from textual.widgets import Static

from src.constants import VBIAS, VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS


class LimitBanner(Static):
    """Read-only summary of bias and vdiff limits."""

    def __init__(self, **kwargs):
        super().__init__(self._text(), **kwargs)

    @staticmethod
    def _text() -> str:
        return (
            f"VBIAS={VBIAS} V   vdiff ∈ [{VDIFF_MIN_VOLTS}, {VDIFF_MAX_VOLTS}] V   "
            "Confirm on connect; never exceed hardware limits."
        )
