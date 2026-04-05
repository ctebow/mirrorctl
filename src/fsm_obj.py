"""
FSM class for python testing dev.
"""

import setup_fsm
import constants
import voltage_helpers as helpers

V_MAX = constants.VDIFF_MAX_VOLTS  # 180


class FSM():

    def __init__(self, slew_time=None, slew_step=None):
        
        self.vdiff_x = 0
        self.vdiff_y = 0
        self.spi = None
        self.enable = None

        if slew_time:
            self.slew_time = slew_time
        else:
            self.slew_time = setup_fsm.SLEW_RATE_MS
        if slew_step:
            self.slew_step = slew_step
        else:
            self.slew_step = setup_fsm.SLEW_AMOUNT_V

    def begin(self) -> int:
        """
        Begin FSM operation. 
        """

        self.vdiff_x = 0.0
        self.vdiff_y = 0.0
        if setup_fsm.IS_LINUX:
            print("LINUX OS DETECTED --> SETTING UP FSM")
            self.spi, self.enable = setup_fsm.fsm_begin()
            if self.spi is not None and self.enable is not None:
                return 0
            else: 
                return -1
        else:
            self.spi = "TEST"
            self.enable = "TEST"
            return 1


    def close(self) -> int:

        if not self.spi:
            print("[FSM Close] FSM not active, nothing to close")
            return

        setup_fsm.fsm_close((self.vdiff_x, self.vdiff_y), (self.slew_time, self.slew_step), self.spi, self.enable)
        self.spi = None
        self.enable = None
        self.vdiff_x = 0
        self.vdiff_y = 0

    def is_active(self) -> bool:

        if self.spi and self.enable:
            return True
        else: return False

    def get_slew_stats(self) -> tuple:
        print(f'Slew rate: {self.slew_time}, Slew step: {self.slew_step}')
        return (self.slew_time, self.slew_step)

    def get_voltages(self) -> tuple:
        return (self.vdiff_x, self.vdiff_y)
    
    def update_slew(self, slew_time, slew_step) -> int:
        self.slew_time = slew_time
        self.slew_step = slew_step
        return 0

    def set_vdiff(self, vdiff_x_new, vdiff_y_new) -> int:

        if vdiff_x_new >= setup_fsm.VDIFF_MAX_VOLTS or vdiff_y_new >= setup_fsm.VDIFF_MAX_VOLTS:
            print("VDIFF TOO HIGH")
            return -1
        if vdiff_x_new <= setup_fsm.VDIFF_MIN_VOLTS or setup_fsm.VDIFF_MIN_VOLTS >= vdiff_y_new:
            print("VDIFF TOO LOW")
            return -1
        
        res =  helpers.slew((self.vdiff_x, self.vdiff_y), (vdiff_x_new, vdiff_y_new), 
                            (self.slew_time, self.slew_step), self.spi)

        if res[0] != vdiff_x_new:
            print("Was not able to fully execute vdiff, perhaps new X was too high")
        if res[1] != vdiff_y_new:
            print("Was not able to fully execute vdiff, perhaps new Y was too high")

        self.vdiff_x = res[0]
        self.vdiff_y = res[1]

        return res


