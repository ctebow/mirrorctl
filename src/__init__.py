# Lazy public API: avoid importing picam (picamera2) or heavy deps at import time.
import importlib

from .constants import VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS

__all__ = [
    "VDIFF_MAX_VOLTS",
    "VDIFF_MIN_VOLTS",
    "FSM",
    "FsmAngular",
    "AxisPolynomialMap",
    "AngularSafetyLimits",
    "centroiding",
    "picam",
]


def __getattr__(name: str):
    if name == "FSM":
        return importlib.import_module(".fsm_obj", __name__).FSM
    if name in {"FsmAngular", "AxisPolynomialMap", "AngularSafetyLimits"}:
        module = importlib.import_module(".fsm_angular", __name__)
        return getattr(module, name)
    if name == "centroiding":
        return importlib.import_module(".centroiding", __name__)
    if name == "picam":
        return importlib.import_module(".picam", __name__)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()) | set(__all__))
