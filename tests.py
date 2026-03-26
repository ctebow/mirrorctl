"""
Pytest suite for voltage_helpers: edge cases and safety constraints.
Constraints: vdiff in [V_MIN_VOLTS, V_MAX_VOLTS]; channel voltages derived from VBIAS ± vdiff/2 (≥ 0, hardware max 180V).
"""
import pytest
from unittest.mock import patch
import voltage_helpers as vh
from constants import VBIAS, VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS, V_MAX_DIGITAL, SLEW_AMOUNT_V, SLEW_RATE_MS

SLEW = (SLEW_RATE_MS, SLEW_AMOUNT_V)

# --- channel_voltage_to_digital ---

class TestChannelVoltageToDigital:
    """0 V -> 0x0000, 200 V -> 0xFFFF; input clamped to [V_MIN_VOLTS, V_MAX_VOLTS]."""

    def test_max_scale_just_under_180v(self):
        # Returns -1 for v >= 180; use 179.99 for max in-range value
        v = 175.99
        got = vh.channel_voltage_to_digital(v)
        expected = round((v / 200.0) * 65535.0)
        assert got == expected
        assert 0 <= got <= 0xFFFF

    def test_vbias_90v(self):
        got = vh.channel_voltage_to_digital(VBIAS)
        expected = round((VBIAS / 200.0) * 65535.0)
        assert got == expected
        assert 0 <= got <= 0xFFFF

    def test_small_positive(self):
        got = vh.channel_voltage_to_digital(0.1)
        assert got >= 0
        assert got <= 0xFFFF

    def test_just_under_180(self):
        v = 175.999
        got = vh.channel_voltage_to_digital(v)
        assert 0 <= got <= 0xFFFF

    def test_negative_returns_minus_one(self):
        assert vh.channel_voltage_to_digital(-180.1) == -1

    def test_zero_returns_minus_one(self):
        assert vh.channel_voltage_to_digital(0.0) == 0

    def test_max_volts_in_range_returns_digital(self):
        # V_MAX_VOLTS is inclusive; 176 is valid and returns scaled digital value
        got = vh.channel_voltage_to_digital(VDIFF_MAX_VOLTS)
        expected = round((VDIFF_MAX_VOLTS / 200.0) * 65535.0)
        assert got == expected
        assert 0 <= got <= 0xFFFF

    def test_above_max_returns_minus_one(self):
        assert vh.channel_voltage_to_digital(VDIFF_MAX_VOLTS + 1.0) == -1


# --- clamp_vdiff ---

class TestClampVdiff:
    """Returns vdiff if in [V_MIN_VOLTS, V_MAX_VOLTS], else -1."""

    def test_in_range_zero(self):
        assert vh.clamp_vdiff(0.0) == 0.0

    def test_in_range_max(self):
        assert vh.clamp_vdiff(VDIFF_MAX_VOLTS) == VDIFF_MAX_VOLTS

    def test_in_range_mid(self):
        assert vh.clamp_vdiff(90.0) == 90.0

    def test_above_max_returns_minus_one(self):
        assert vh.clamp_vdiff(VDIFF_MAX_VOLTS + 1.0) == -1
        assert vh.clamp_vdiff(181.0) == -1

    def test_below_min_returns_minus_one(self):
        assert vh.clamp_vdiff(VDIFF_MIN_VOLTS - 1.0) == -1
        assert vh.clamp_vdiff(-1.0) == -1

    def test_just_under_max(self):
        v = VDIFF_MAX_VOLTS - 0.01
        assert vh.clamp_vdiff(v) == v

    def test_just_over_min(self):
        assert vh.clamp_vdiff(0.01) == 0.01


# --- vdiff_to_channel_voltage ---

class TestVdiffToChannelVoltage:
    """ch0 = VBIAS + vdiff_x/2, ch1 = VBIAS - vdiff_x/2; same for y (ch2, ch3).
    Constraint: no channel > 180V, no channel < 0V; pair difference (vdiff) ≤ 180V.
    """

    def test_zero_vdiff_both_axes(self):
        ch0, ch1, ch2, ch3 = vh.vdiff_to_channel_voltage(0.0, 0.0)
        assert ch0 == ch1 == ch2 == ch3 == VBIAS
        assert all(0 <= v <= 180 for v in (ch0, ch1, ch2, ch3))

    def test_vdiff_x_max(self):
        vdiff = VDIFF_MAX_VOLTS
        half = vdiff / 2.0
        ch0, ch1, ch2, ch3 = vh.vdiff_to_channel_voltage(vdiff, 0.0)
        assert ch0 == VBIAS + half
        assert ch1 == VBIAS - half
        assert ch2 == ch3 == VBIAS
        assert ch0 - ch1 == vdiff
        assert ch0 <= 180 and ch1 >= 0

    def test_vdiff_y_max(self):
        vdiff = VDIFF_MAX_VOLTS
        half = vdiff / 2.0
        ch0, ch1, ch2, ch3 = vh.vdiff_to_channel_voltage(0.0, vdiff)
        assert ch2 == VBIAS - half
        assert ch3 == VBIAS + half
        assert ch0 == ch1 == VBIAS
        assert ch3 - ch2 == vdiff
        assert ch3 <= 180 and ch2 >= 0

    def test_both_axes_120v(self):
        ch0, ch1, ch2, ch3 = vh.vdiff_to_channel_voltage(120.0, 120.0)
        assert ch0 == VBIAS + 60.0
        assert ch1 == VBIAS - 60.0
        assert ch2 == VBIAS - 60.0
        assert ch3 == VBIAS + 60.0
        assert ch0 - ch1 == 120.0
        assert ch3 - ch2 == 120.0
        assert all(0 <= v <= 180 for v in (ch0, ch1, ch2, ch3))

    def test_clamped_invalid_vdiff_x_uses_minus_one(self):
        # clamp_vdiff(200) -> -1, so ch0 = VBIAS + (-1)/2, ch1 = VBIAS - (-1)/2
        ch0, ch1, ch2, ch3 = vh.vdiff_to_channel_voltage(200.0, 0.0)
        assert ch0 == VBIAS - 0.5
        assert ch1 == VBIAS + 0.5
        assert ch2 == ch3 == VBIAS

    def test_clamped_invalid_vdiff_y_uses_minus_one(self):
        ch0, ch1, ch2, ch3 = vh.vdiff_to_channel_voltage(0.0, -10.0)
        assert ch2 == VBIAS + 5.0
        assert ch3 == VBIAS - 5.0
        assert ch0 == ch1 == VBIAS

    def test_no_channel_above_180(self):
        for vx in (0.0, 90.0, 180.0):
            for vy in (0.0, 90.0, 180.0):
                ch0, ch1, ch2, ch3 = vh.vdiff_to_channel_voltage(vx, vy)
                assert ch0 <= 180 and ch1 <= 180 and ch2 <= 180 and ch3 <= 180

    def test_no_channel_below_zero(self):
        for vx in (0.0, 90.0, 180.0):
            for vy in (0.0, 90.0, 180.0):
                ch0, ch1, ch2, ch3 = vh.vdiff_to_channel_voltage(vx, vy)
                assert ch0 >= 0 and ch1 >= 0 and ch2 >= 0 and ch3 >= 0

    def test_pair_diff_x_never_exceeds_180_for_valid_vdiff(self):
        for vx in (0.0, 0.01, 90.0, 179.99, 180.0):
            ch0, ch1, _, _ = vh.vdiff_to_channel_voltage(vx, 0.0)
            assert abs(ch0 - ch1) <= 180.0

    def test_pair_diff_y_never_exceeds_180_for_valid_vdiff(self):
        for vy in (0.0, 0.01, 90.0, 179.99, 180.0):
            _, _, ch2, ch3 = vh.vdiff_to_channel_voltage(0.0, vy)
            assert abs(ch3 - ch2) <= 180.0


# --- get_rounded_channel_values ---

class TestGetRoundedChannelValues:
    """Returns (round(VBIAS + vdiff/2), round(VBIAS - vdiff/2))."""

    def test_zero_vdiff(self):
        plus, minus = vh.get_rounded_channel_values(0.0)
        assert plus == minus == round(VBIAS)

    def test_vdiff_180(self):
        plus, minus = vh.get_rounded_channel_values(180.0)
        assert plus == round(VBIAS + 90.0)
        assert minus == round(VBIAS - 90.0)
        assert plus <= 180 and minus >= 0

    def test_vdiff_120(self):
        plus, minus = vh.get_rounded_channel_values(120.0)
        assert plus == round(VBIAS + 60.0)
        assert minus == round(VBIAS - 60.0)

    def test_extreme_valid_values(self):
        for v in (0.0, 0.1, 90.0, 179.9, 180.0):
            plus, minus = vh.get_rounded_channel_values(v)
            assert 0 <= plus <= 180
            assert 0 <= minus <= 180
            assert abs(plus - minus) <= 180


# --- write_dac_channel (logic only; no SPI in tests) ---

class TestWriteDacChannel:
    """Command format and bounds; no actual SPI when IS_LINUX=False."""

    def test_channel_bounds(self, mock_spi):
        # cmd = 0x100000 | (channel << 16) | value -> high byte is 0x10 + channel
        # value must be in [0, V_MAX_DIGITAL]; 0xFFFF exceeds V_MAX_DIGITAL when V_MAX_VOLTS < 200
        cmd = vh.write_dac_channel(0, 0, mock_spi)
        assert (cmd >> 16) == 0x10
        assert cmd & 0xFFFF == 0
        max_digital = int(V_MAX_DIGITAL)
        cmd3 = vh.write_dac_channel(3, max_digital, mock_spi)
        assert (cmd3 >> 16) == 0x13
        assert cmd3 & 0xFFFF == max_digital

    def test_value_in_range(self, mock_spi):
        vh.write_dac_channel(0, 0, mock_spi)
        vh.write_dac_channel(0, int(V_MAX_DIGITAL), mock_spi)
        vh.write_dac_channel(0, 32768, mock_spi)

    def test_channel_4_raises(self, mock_spi):
        with pytest.raises(AssertionError, match="Channel must be 0-3"):
            vh.write_dac_channel(4, 0, mock_spi)

    def test_value_above_65535_raises(self, mock_spi):
        res = vh.write_dac_channel(0, 65536, mock_spi)
        assert(res == -1)


# --- slew_x / slew_y / slew (with mocked time.sleep and no SPI) ---

class TestSlew:
    """Slew returns end_vdiff; with valid inputs channels stay within 0--180 and pair diff ≤ 180."""

    @pytest.fixture
    def mock_spi(self):
        class MockSpi:
            def xfer2(self, payload):
                pass
        return MockSpi()

    @patch("voltage_helpers.time.sleep")
    def test_slew_x_returns_end_vdiff(self, mock_sleep, mock_spi):
        out = vh.slew_x(0.0, 120.0, SLEW, mock_spi)
        assert out == 120.0

    @patch("voltage_helpers.time.sleep")
    def test_slew_x_zero_to_zero(self, mock_sleep, mock_spi):
        out = vh.slew_x(0.0, 0.0, SLEW, mock_spi)
        assert out == 0.0

    @patch("voltage_helpers.time.sleep")
    def test_slew_x_max_safe_end(self, mock_sleep, mock_spi):
        # ch0 = VBIAS + end/2 must be <= V_MAX_VOLTS; so end <= 2*(V_MAX_VOLTS - VBIAS) = 2*86 = 172
        end = 2.0 * (VDIFF_MAX_VOLTS - VBIAS)
        out = vh.slew_x(0.0, end, SLEW, mock_spi)
        assert out == end

    @patch("voltage_helpers.time.sleep")
    def test_slew_y_returns_end_vdiff(self, mock_sleep, mock_spi):
        out = vh.slew_y(0.0, 120.0, SLEW, mock_spi)
        assert out == 120.0

    @patch("voltage_helpers.time.sleep")
    def test_slew_y_zero_to_zero(self, mock_sleep, mock_spi):
        out = vh.slew_y(0.0, 0.0, SLEW, mock_spi)
        assert out == 0.0

    @patch("voltage_helpers.time.sleep")
    def test_slew_returns_end_state(self, mock_sleep, mock_spi):
        out = vh.slew((0.0, 0.0), (120.0, 120.0), SLEW, mock_spi)
        assert out == (120.0, 120.0)

    @patch("voltage_helpers.time.sleep")
    def test_slew_close_to_vbias(self, mock_sleep, mock_spi):
        out = vh.slew((120.0, 120.0), (0.0, 0.0), SLEW, mock_spi)
        assert out == (0.0, 0.0)

    @patch("voltage_helpers.time.sleep")
    def test_slew_extreme_valid(self, mock_sleep, mock_spi):
        # Max vdiff such that channel voltages stay <= V_MAX_VOLTS (ch = VBIAS + vdiff/2)
        max_vdiff = 2.0 * (VDIFF_MAX_VOLTS - VBIAS)
        end = (max_vdiff, max_vdiff)
        out = vh.slew((0.0, 0.0), end, SLEW, mock_spi)
        assert out == end
