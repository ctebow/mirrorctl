#!/usr/bin/env python3
"""Read ADS1115 inputs A0-A3 using only the Python standard library.

No Blinka / Adafruit_BBIO needed: talks to the chip via /dev/i2c-N.

SHOULD ALSO WORK FOR RASPBERRY PI, BUT MAKE SURE I2C IS ENABLED
"""
import fcntl
import os
import struct
import time

# IF I2C ISN"T WORKING, USE I2C DETECT OR GOOGLE "HOW TO ENABLE I2C ONE ON
# RASPBERRY PI 4B" (it will probably need to be enabled)

I2C_BUS = 1        # bus where `i2cdetect -y -r 2` showed 48
ADDR = 0x48
I2C_SLAVE = 0x0703  # ioctl to select the device address

# PGA setting -> full-scale voltage
PGA = 0b001        # +/-4.096 V (good for 0-3.3 V signals)
FSR = 4.096
DR = 0b100         # 128 samples per second

REG_CONVERSION = 0x00
REG_CONFIG = 0x01


def adc_init():
    fd = os.open(f"/dev/i2c-{I2C_BUS}", os.O_RDWR)
    fcntl.ioctl(fd, I2C_SLAVE, ADDR)
    return fd

def adc_close(adc):
    os.close(adc)


def _read_channel(ch, fd):
    """Single-shot, single-ended read of channel 0-3. Returns (raw, volts)."""
    config = (
        0x8000                 # OS: start a conversion
        | ((0b100 + ch) << 12) # MUX: AINch vs GND
        | (PGA << 9)
        | 0x0100               # single-shot mode
        | (DR << 5)
        | 0x0003               # comparator disabled
    )
    os.write(fd, bytes([REG_CONFIG, config >> 8, config & 0xFF]))

    # Wait for the conversion to finish (OS bit goes back to 1)
    deadline = time.time() + 0.1
    while True:
        os.write(fd, bytes([REG_CONFIG]))
        if os.read(fd, 2)[0] & 0x80:
            break
        if time.time() > deadline:
            raise TimeoutError(f"ADS1115 conversion timed out on A{ch}")
        time.sleep(0.001)

    os.write(fd, bytes([REG_CONVERSION]))
    raw = struct.unpack(">h", os.read(fd, 2))[0]
    return raw, raw * FSR / 32768

def get_centroid_err_qpd(fd, delay_ms=0, channels=4):
    """
    Calculate centroid error using read_channel and dac_init. Based on QPD
    quadrant formula. 
    """
    quads = []
    for ch in range(channels):
        raw, volts = _read_channel(ch, fd)
        quads.append(volts)
    
    # FROM LEFT TO RIGHT, TOP TO BOTTOM: 
    a = quads[0]
    b = quads[1]
    c = quads[3] # manufacturer has them flipped i think, otherwise 
    d = quads[2]

    err_x = ((b + d) - (a + c)) / (a + b + c + d) # normalized
    err_y = ((a + b) - (c + d)) / (a + b + c + d)

    if delay_ms != 0:
        time.sleep(delay_ms)

    return err_x, err_y
