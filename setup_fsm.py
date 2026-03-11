"""
Setup and shutdown sequences for Mirrorcle FSM. ORDER IS VERY IMPORTANT HERE,
FOLLOW THE DIRECTIONS. 

Setup order:
1. Pin setup (GPIO for DRIVER_ENABLE and optional CS; PWM for FCLK).
2. DRIVER_ENABLE LOW; CS inactive; open SPI.
3. DAC init sequence (four 24-bit commands with 10 ms delays).
4. Set DAC channels 0–3 to VBIAS, 0, VBIAS, 0 with 10 ms between writes.
5. Start filter clock PWM (30 kHz, 50% duty).
6. DRIVER_ENABLE HIGH.
"""
IS_LINUX = True
try:
    import spidev
    import pigpio
except ImportError:
    IS_LINUX = False
import time
import voltage_helpers as helpers

# filter clock pins
FCLK_PWM_PIN_1 = 12  # Must be a PWM-capable pin; config-pin to pwm if needed
FCLK_PWM_PIN_2 = 13
FCLK_HZ = 30_000
FCLK_DUTY_PERCENT = 500000

# delay
DELAY_MS = 10
DELAY_S = DELAY_MS / 1000.0

# voltages
VBIAS = 90.0  # Volts; 0 < VBIAS <= 200 FOR 90V VBIAS MAX VDIFF IS 180V
VDIFF_MAX_VOLTS = 177 # Driver can do max 200V, current mirror is 180V
VDIFF_MIN_VOLTS = -177
V_MAX_CHANNEL= VBIAS + VDIFF_MAX_VOLTS / 2
V_MAX_DIGITAL = (V_MAX_CHANNEL / 200) * 65535.0


SLEW_RATE_MS = 1 / 1000.0
SLEW_AMOUNT_V = 0.25 # in units of volts

# DAC to FSM enable
#DAC_ENABLE_CHIP = "gpiochip0"
DAC_ENABLE_LINE = 6

# DAC setup 
DAC_RESET = 0x280001
DAC_ENABLE_INTERFACE = 0x380001
DAC_ENABLE_CHANNELS = 0x20000F
DAC_ENABLE_SOFTWARE = 0x300000

# DAC channels
DAC_CH0 = 0
DAC_CH1 = 1
DAC_CH2 = 2
DAC_CH3 = 3

# SPI setup
SPI_MODE = 0b01
SPI_MAX_SPEED = 1_000_000


def fsm_begin() -> tuple:
    """
    Run the exact Mirrorcle driver setup sequence (order-sensitive).
    Returns SPI and Line object for use by other functions. 
    """

    print(f'FSM Init: VBIAS: {VBIAS} -- MAX VDIFF: {VDIFF_MAX_VOLTS} -- MIN VDIFF: {VDIFF_MIN_VOLTS} -- SLEW RATE: {SLEW_RATE_MS}')
    print(f'If these settings look correct, input "Y" to continue. Continuing will result in FSM being active at {VBIAS}V VBIAS. Press any other key to quit')
    resp = input("continue: ")
    if (resp != "Y"):
        return (None, None)
    
    print("FSM Begin")
    if IS_LINUX:
        print("CREATING SPI")
        # Create enable line and write to low
        pi = pigpio.pi()
        pi.set_mode(DAC_ENABLE_LINE, pigpio.OUTPUT)
        pi.write(DAC_ENABLE_LINE, 0)

        # begin SPI connection
        spi = spidev.SpiDev()
        spi.open(0,0)
        spi.max_speed_hz = SPI_MAX_SPEED
        spi.mode = SPI_MODE

    # DAC initialization sequence (10 ms after each command)
    print("WRITING DAC INIT")
    helpers.send_dac_command(spi, DAC_RESET)  # Full reset
    time.sleep(DELAY_S)
    helpers.send_dac_command(spi, DAC_ENABLE_INTERFACE)  # Enable internal reference
    time.sleep(DELAY_S)
    helpers.send_dac_command(spi, DAC_ENABLE_CHANNELS)  # Enable all channels
    time.sleep(DELAY_S)
    helpers.send_dac_command(spi, DAC_ENABLE_SOFTWARE)  # Enable software LDAC
    time.sleep(DELAY_S)

    # Set VBIAS on ALL channels
    digital_bias = helpers.channel_voltage_to_digital(VBIAS)

    helpers.write_dac_channel(DAC_CH0, digital_bias, spi)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(DAC_CH1, digital_bias ,spi)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(DAC_CH2, digital_bias, spi)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(DAC_CH3, digital_bias, spi)
    time.sleep(DELAY_S)
    if IS_LINUX:
        # Set Filter clock: 30 kHz, 50% duty --> Two of these jawns
        #PWM.start(FCLK_PWM_PIN_1, FCLK_DUTY_PERCENT, FCLK_HZ)
        #PWM.start(FCLK_PWM_PIN_2, FCLK_DUTY_PERCENT, FCLK_HZ)
        #TODO: pigpio add here. 
        pi.hardware_PWM(FCLK_PWM_PIN_1, FCLK_HZ, FCLK_DUTY_PERCENT)
        pi.hardware_PWM(FCLK_PWM_PIN_2, FCLK_HZ, FCLK_DUTY_PERCENT)


        # 6) Enable driver
        pi.write(DAC_ENABLE_LINE, 1)
        time.sleep(DELAY_S)

        return spi, pi
    return None, None

def fsm_close(start_state, slew_params, spi, pi):
    

    # slew to zero vdiff (all channels at VBIAS)
    helpers.slew(start_state, (0.0, 0.0), slew_params, spi)

    # recalculate digital bias
    digital_bias = helpers.channel_voltage_to_digital(VBIAS)
    print("Digital Bias:", digital_bias)



    # set fsm back to vbias just in case
    helpers.write_dac_channel(0, digital_bias, spi)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(1, digital_bias, spi)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(2, digital_bias, spi)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(3, digital_bias, spi)
    time.sleep(DELAY_S)

    if IS_LINUX:
        # disable dac
        pi.write(DAC_ENABLE_LINE, 0)
        time.sleep(DELAY_S)

        #close spi connection
        spi.close()

