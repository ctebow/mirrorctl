# filter clock pins
FCLK_PWM_PIN_1 = 12  # Must be a PWM-capable pin; config-pin to pwm if needed
FCLK_PWM_PIN_2 = 13
FCLK_HZ = 6_000 # (8_400 for 5mm)should be 60 * 6th order bessel LPF cutoff (140HZ for 5mm diamter mirror, I think 100 for 6.4mm)
FCLK_DUTY_PERCENT = 500000

# delay
DELAY_MS = 10
DELAY_S = DELAY_MS / 1000.0

# voltages --> BEST PRACTICE TO KEEP VBIAS BELOW 170
VBIAS = 90.0  # Volts; 0 < VBIAS <= 200 FOR 90V VBIAS MAX VDIFF IS 170V
VDIFF_MAX_VOLTS = 170 # Driver can do max 200V, current mirror is 170V
VDIFF_MIN_VOLTS = -170
V_MAX_CHANNEL= VBIAS + VDIFF_MAX_VOLTS / 2
V_MAX_DIGITAL = (V_MAX_CHANNEL / 200) * 65535.0


SLEW_RATE_MS = 1 / 10000.0 #take off one zero for slower
SLEW_AMOUNT_V = 0.25 # in units of volts

# DAC to FSM enable
#DAC_ENABLE_CHIP = "gpiochip0" (for bb not pi) 
# the enable line CAN MISBEHAVE if you use wrong gpio pin, this would be very bad if connected to mirror
DAC_ENABLE_LINE = 16  #used to b 6, but we need this pin to be default low (rpi gpio pins 9+)

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

CURRENT_SCREEN_DISTANCE = 360 # mm

# centroiding threshold configuration
LASER_CENTROID_THRESHOLD_ENABLED = True
LASER_CENTROID_INTENSITY_THRESHOLD = 0

# camera defaults for mapping/centroiding
CAMERA_DEFAULT_WIDTH = 1280 #640
CAMERA_DEFAULT_HEIGHT = 720 #480
CAMERA_FRAME_RATE = 60.0
CAMERA_MAIN_FORMAT = "RGB888"

# exposure/gain controls
CAMERA_AUTO_EXPOSURE_ENABLED = False
CAMERA_EXPOSURE_TIME_US = 200
CAMERA_ANALOGUE_GAIN = 1.0

# white balance controls
CAMERA_AUTO_WHITE_BALANCE_ENABLED = False
CAMERA_COLOUR_GAINS = (1.0, 1.0)