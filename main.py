#!/usr/bin/env python3
"""
Mirrorcle MEMS Digital Driver — BeagleBone Python port.

TRIPLE CHECK BEFORE RUNNING WITH ACTUAL MEMS MIRROR.
VDIFF (Channel 0 - Channel 1 or Channel 2 - Channel 3) MUST NOT EXCEED 2 * VBIAS
(e.g. 180 V for VBIAS=90). Exceeding this can permanently damage the mirror.
Do not use with the mirror before checking with Graydon, Sofi, or Caden.

Wiring (BeagleBone):
- DAC SYNC: P9_17 (SPI0 CS0) if CS_PIN is None; else wire to CS_PIN.
- DAC MOSI/SCLK: P9_18 (DI), P9_22 (SCLK) for SPI0.
- DRIVER_ENABLE: configurable (e.g. P8_14).
- FCLK: configurable PWM (e.g. P9_14); run `config-pin P9_14 pwm` if needed.


SOFTWARE:
- All control flow functions assume the mirror is starting at 90VBIAS by setting 
it to 90VBIAS at beginning using slew.
"""

from fsm_obj import FSM

if __name__ == "__main__": 

    # THIS IS WHERE ALL SETUP HAPPENS --> FSM IS NOW ACTIVE AT 90VBIAS
    fsm = FSM()
    active = fsm.begin()
    print(fsm.get_voltages())

    if active != 1:
        try:
            while True:  # main loop
                usr = input("Input vdiffx, vdiffy (x y): ")
                lst = usr.split()
                if len(lst) != 2:
                    if lst[0] == "sin":
                        fsm.drive_sine(10, 2, 10)
                    else:
                        continue
                x, y = tuple(lst)
                if x and y:
                    print(f'setting vdiff: x: {x}, y: {y}')
                    fsm.set_vdiff(float(x), float(y))
                else:
                    continue

        except KeyboardInterrupt:
            print("Keyboard interrupt received — shutting down")
        except:
            print("Other error occurred, shutting down.")
        finally:
            fsm.close()

    else:
        print("FSM in test mode, not on linux system. Wanna try slew?: ")
        try:
            while True:  # main loop
                    usr = input("Input vdiffx, vdiffy (x y): ")
                    lst = usr.split()
                    if len(lst) != 2:
                        if lst[0] == "sin":
                            fsm.drive_sine(10, 2, 10)
                        else:
                            continue
                    x, y = tuple(lst)
                    if x and y:
                        print(f'setting vdiff: x: {x}, y: {y}')
                        fsm.set_vdiff(float(x), float(y))
                    else:
                        continue

        except KeyboardInterrupt:
            print("Keyboard interrupt received — shutting down")
        except:
            print("Other error occurred, shutting down.")
        finally:
            fsm.close()

        

