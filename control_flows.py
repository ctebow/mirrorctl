"""
Functions implementing various mirror control patterns.

Step between two points, do a quasi-static scan, etc. 

ALL CONTROL FLOW FUNCTIONS SHOULD RETURN FINAL CHANNEL VALUES IN TUPLE
(0, 1, 2, 3). Allow FSM obj to reset the position. 
"""


import time
import voltage_helpers as helpers
from constants import VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS, VBIAS, DELAY_S

def angle_to_angle(x: float, y: float) -> None:
    raise NotImplementedError

def set_to_vbias() -> None:
    raise NotImplementedError



def get_voltages():
    print("Input voltages: ")
    usr_input = input()
    voltages = usr_input.split(" ")
    if len(voltages) > 2:
        print("improper format: <vDiffX vDiffY> ")
    vX = int(voltages[0])
    vY = int(voltages[1])
    if vX >=VDIFF_MAX_VOLTS or vX < VDIFF_MIN_VOLTS:
        print("X voltage not in range, try again")
    if vY >=VDIFF_MAX_VOLTS or vY < VDIFF_MIN_VOLTS:
        print("Y voltage not in range, try again")

    return vX, vY
    
def set_bias(vX, vY):
    vX_digital = helpers.vbias_to_dac(vX)
    vY_digital = helpers.vbias_to_dac(vY)
    print(vX_digital, vY_digital)

    helpers.write_dac_channel(0, vX_digital)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(1, 0)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(2, vY_digital)
    time.sleep(DELAY_S)
    helpers.write_dac_channel(3, 0)
    time.sleep(DELAY_S)