"""Domain exceptions for hardware and calibration (no printing in library code)."""


class MirrorctlError(Exception):
    """Base class for mirrorctl errors."""


class HardwareUnavailable(MirrorctlError):
    """SPI, GPIO, or camera not available or failed to open."""


class UnsafeVoltageRequest(MirrorctlError):
    """Requested vdiff or channel voltage outside allowed range."""

    def __init__(self, message: str, *, axis: str | None = None):
        super().__init__(message)
        self.axis = axis


class InitializationAborted(MirrorctlError):
    """User or policy declined hardware initialization (e.g. confirm false)."""


class CalibrationError(MirrorctlError):
    """Missing or invalid calibration data."""
