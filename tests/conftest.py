"""
Pytest configuration: disable SPI and speed up slew tests.
"""
import pytest


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    """Ensure ``src.voltage_helpers`` does not use real SPI (IS_LINUX=False for DAC writes)."""
    import src.voltage_helpers as vh

    monkeypatch.setattr(vh, "IS_LINUX", False)


@pytest.fixture
def mock_spi():
    """Dummy SPI object for slew tests (xfer2 is not called when IS_LINUX=False)."""

    class MockSpi:
        def xfer2(self, payload):
            pass

    return MockSpi()
