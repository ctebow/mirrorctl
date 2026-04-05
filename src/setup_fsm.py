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
from constants import FCLK_DUTY_PERCENT, FCLK_HZ, FCLK_PWM_PIN_1, FCLK_PWM_PIN_2, DELAY_MS, DELAY_S, VBIAS, VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS, V_MAX_DIGITAL, V_MAX_CHANNEL, SLEW_AMOUNT_V, SLEW_RATE_MS, DAC_ENABLE_LINE, DAC_ENABLE_CHANNELS, DAC_ENABLE_INTERFACE, DAC_ENABLE_SOFTWARE, DAC_RESET, DAC_CH0, DAC_CH1, DAC_CH2, DAC_CH3, SPI_MAX_SPEED, SPI_MODE


def fsm_begin() -> tuple:
    """
    Run the exact Mirrorcle driver setup sequence (order-sensitive).
    Returns SPI and Line object for use by other functions. 
    """

    print(f'FSM Init: VBIAS: {VBIAS} -- MAX VDIFF: {VDIFF_MAX_VOLTS} -- MIN VDIFF: {VDIFF_MIN_VOLTS} -- SLEW RATE: {SLEW_RATE_MS}')
    print(f'If these settings look correct, input "Y" to continue. Continuing will result in FSM being active at {VBIAS}V VBIAS. Press any other key to quit')
    resp = input("continue?: ")
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

        print("Shut down FSM")

