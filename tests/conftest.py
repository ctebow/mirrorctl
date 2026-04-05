"""
Pytest configuration: disable SPI and speed up slew tests.
"""
import pytest
import sys


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    """Ensure voltage_helpers does not use real SPI (IS_LINUX=False for DAC writes)."""
    import voltage_helpers
    monkeypatch.setattr(voltage_helpers, "IS_LINUX", False)


@pytest.fixture
def mock_spi():
    """Dummy SPI object for slew tests (xfer2 is not called when IS_LINUX=False)."""
    class MockSpi:
        def xfer2(self, payload):
            pass
    return MockSpi()
