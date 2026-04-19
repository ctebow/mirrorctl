"""
Helper functions for translating decimal voltages to digital hex values for 
quad-DAC. New Philosophy: User only inputs vbias, everything else is taken caer 
of under the hood. 

ADD MIRRORCLE DOCUMENTATION DESCRIBING HOW TO CONVERT. 
"""
import time
from .constants import V_MAX_CHANNEL, VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS, VBIAS, V_MAX_DIGITAL

IS_LINUX = True


def channel_voltage_to_digital(volts: float) -> int:
    """
    Convert absolute channel voltage (0–200 V scale) to 16-bit DAC code.
    Values outside the allowed channel range for this hardware return -1 (no silent vdiff clamping).
    """
    v = float(volts)
    if v < 0 or v > V_MAX_CHANNEL:
        return -1
    return round((v / 200.0) * 65535.0)


def send_dac_command(spi, cmd: int) -> None:
    """Send a 24-bit command to the DAC. CS asserted for full transfer."""
    payload = [(cmd >> 16) & 0xFF, (cmd >> 8) & 0xFF, cmd & 0xFF]
    spi.xfer2(payload)
    
# (VdifferenceMax/200)*65565

def write_dac_channel(channel: int, value: int, spi) -> int:
    """Write 16-bit value to DAC channel (0=A, 1=B, 2=C, 3=D)."""
    assert(0 <= channel <= 3), "ERROR: Channel must be 0-3"
    if value < (0) or value > V_MAX_DIGITAL:
        return -1
    cmd = 0x100000 | (channel << 16) | (int(value) & 0xFFFF)
    if IS_LINUX:
        send_dac_command(spi, cmd)
    return cmd

def clamp_vdiff(vdiff: float) -> float:
    if vdiff > VDIFF_MAX_VOLTS or vdiff < VDIFF_MIN_VOLTS:
        return -1
    else: return vdiff

def vdiff_to_channel_voltage(vdiff_x: float, vdiff_y: float) -> tuple:
    vdiff_x = clamp_vdiff(vdiff_x)
    vdiff_y = clamp_vdiff(vdiff_y)
    ch0 = VBIAS + vdiff_x / 2
    ch1 = VBIAS - vdiff_x / 2
    ch2 = VBIAS - vdiff_y / 2
    ch3 = VBIAS + vdiff_y / 2
    return (ch0, ch1, ch2, ch3)

def get_rounded_channel_values(vdiff: float) -> int:

    ch_plus = round(VBIAS + vdiff / 2)
    ch_minus = round(VBIAS - vdiff / 2)

    return (ch_plus, ch_minus)

def slew_x(start_vdiff: float, end_vdiff: float, slew_params: tuple, spi) -> float:

    slew_time, slew_step = slew_params
    total = end_vdiff - start_vdiff
    sign = 1
    if (total < 0):
        sign = -1
    total = abs(total)
    slewed = 0
    curr_vdiff = start_vdiff
    while slewed < total:
        
        ch0_v, ch1_v, _, _ = vdiff_to_channel_voltage(curr_vdiff, 0)
        if abs(ch0_v - ch1_v) > 180:
            break
        if (total - slewed) < slew_step:
            ch0_v, ch1_v, _, _ = vdiff_to_channel_voltage(end_vdiff, 0)
            ch0_dac = channel_voltage_to_digital(ch0_v)
            ch1_dac = channel_voltage_to_digital(ch1_v)

            if (ch0_dac == -1 or ch1_dac == -1):
                return curr_vdiff

            ok1 = write_dac_channel(0, ch0_dac, spi)
            ok2 = write_dac_channel(1, ch1_dac, spi)
            if ok1 == -1 or ok2 == -1:
                return curr_vdiff
            break
        ch0_dac = channel_voltage_to_digital(ch0_v)
        ch1_dac = channel_voltage_to_digital(ch1_v)
        if (ch0_dac == -1 or ch1_dac == -1):
            print(f'channel_voltage_to_digital early return')
            return curr_vdiff
        ok1 = write_dac_channel(0, ch0_dac, spi)
        ok2 = write_dac_channel(1, ch1_dac, spi)
        if ok1 == -1 or ok2 == -1:
            print(f'Write dac channel early return')
            return curr_vdiff

        curr_vdiff += slew_step * sign
        slewed += slew_step
        time.sleep(slew_time)

    ch0_v_final, ch1_v_final, _, _ = vdiff_to_channel_voltage(end_vdiff, 0)
    ch0_dac_final = channel_voltage_to_digital(ch0_v_final)
    ch1_dac_final = channel_voltage_to_digital(ch1_v_final)
    write_dac_channel(0, ch0_dac_final, spi)
    write_dac_channel(1, ch1_dac_final, spi)
    print(f"Wrote final vdiff_x: {end_vdiff} -> ch0: {ch0_v_final} ch1: {ch1_v_final}")
    return end_vdiff



def slew_y(start_vdiff: float, end_vdiff: float, slew_params: tuple, spi) -> None:

    slew_time, slew_step = slew_params
    total = end_vdiff - start_vdiff
    sign = 1
    if (total < 0):
        sign = -1
    total = abs(total)
    slewed = 0
    curr_vdiff = start_vdiff
    while slewed < total:
        
        _, _, ch2_v, ch3_v = vdiff_to_channel_voltage(0, curr_vdiff)
        if abs(ch2_v - ch3_v) > 180:
            break
        if (total - slewed) < slew_step:
            _, _, ch2_v, ch3_v = vdiff_to_channel_voltage(0, end_vdiff)
            ch2_dac = channel_voltage_to_digital(ch2_v)
            ch3_dac = channel_voltage_to_digital(ch3_v)
            if ch2_dac == -1 or ch3_dac == -1:
                return curr_vdiff
            ok2 = write_dac_channel(2, ch2_dac, spi)
            ok3 = write_dac_channel(3, ch3_dac, spi)
            if (ok2 == -1 or ok3 == -1):
                return curr_vdiff
            break
        ch2_dac = channel_voltage_to_digital(ch2_v)
        ch3_dac = channel_voltage_to_digital(ch3_v)
        if ch2_dac == -1 or ch3_dac == -1:
            return curr_vdiff
        ok2 = write_dac_channel(2, ch2_dac, spi)
        ok3 = write_dac_channel(3, ch3_dac, spi)
        if ok2 == -1 or ok3 == -1:
            return curr_vdiff

        curr_vdiff += slew_step * sign
        slewed += slew_step
        time.sleep(slew_time)

    _, _, ch2_v_final, ch3_v_final= vdiff_to_channel_voltage(0, end_vdiff)
    ch2_dac_final = channel_voltage_to_digital(ch2_v_final)
    ch3_dac_final = channel_voltage_to_digital(ch3_v_final)
    write_dac_channel(2, ch2_dac_final, spi)
    write_dac_channel(3, ch3_dac_final, spi)
    print(f"Wrote final vdiff_y: {end_vdiff} -> ch2: {ch2_v_final} ch3: {ch3_v_final}")

    return end_vdiff


def slew(start_state: tuple, end_state: tuple, slew_params: tuple, spi) -> tuple:

    vdiff_x_start, vdiff_y_start = start_state
    vdiff_x_end, vdiff_y_end = end_state
    vdiff_x_end = slew_x(vdiff_x_start, vdiff_x_end, (slew_params), spi)
    vdiff_y_end = slew_y(vdiff_y_start, vdiff_y_end, (slew_params), spi)
    return (vdiff_x_end, vdiff_y_end)


def slew_xy_coordinated(start_state: tuple, end_state: tuple, slew_params: tuple, spi) -> tuple:
    """
    Coordinated XY slew: step x/y together so both axes progress concurrently.

    Uses linear interpolation across a shared number of steps determined from the
    larger axis move and ``slew_step``. Safety behavior mirrors existing helpers:
    invalid channel voltages or DAC writes abort and return the last commanded
    vdiff tuple.
    """
    vdiff_x_start, vdiff_y_start = start_state
    vdiff_x_end, vdiff_y_end = end_state
    slew_time, slew_step = slew_params

    dx = vdiff_x_end - vdiff_x_start
    dy = vdiff_y_end - vdiff_y_start
    max_delta = max(abs(dx), abs(dy))

    if max_delta == 0:
        return (vdiff_x_start, vdiff_y_start)
    if slew_step <= 0:
        return (vdiff_x_start, vdiff_y_start)

    steps = max(1, int(max_delta / slew_step))
    if (max_delta - (steps * slew_step)) > 0:
        steps += 1

    last_x = vdiff_x_start
    last_y = vdiff_y_start

    for i in range(1, steps + 1):
        frac = i / steps
        curr_x = vdiff_x_start + (dx * frac)
        curr_y = vdiff_y_start + (dy * frac)

        ch0_v, ch1_v, ch2_v, ch3_v = vdiff_to_channel_voltage(curr_x, curr_y)
        if abs(ch0_v - ch1_v) > 180 or abs(ch2_v - ch3_v) > 180:
            return (last_x, last_y)

        ch0_dac = channel_voltage_to_digital(ch0_v)
        ch1_dac = channel_voltage_to_digital(ch1_v)
        ch2_dac = channel_voltage_to_digital(ch2_v)
        ch3_dac = channel_voltage_to_digital(ch3_v)
        if ch0_dac == -1 or ch1_dac == -1 or ch2_dac == -1 or ch3_dac == -1:
            return (last_x, last_y)

        ok0 = write_dac_channel(0, ch0_dac, spi)
        ok1 = write_dac_channel(1, ch1_dac, spi)
        ok2 = write_dac_channel(2, ch2_dac, spi)
        ok3 = write_dac_channel(3, ch3_dac, spi)
        if ok0 == -1 or ok1 == -1 or ok2 == -1 or ok3 == -1:
            return (last_x, last_y)

        last_x = curr_x
        last_y = curr_y
        time.sleep(slew_time)

    return (vdiff_x_end, vdiff_y_end)
