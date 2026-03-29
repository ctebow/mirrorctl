
import time
import csv
import click

import centroiding
import picam
from constants import VDIFF_MAX_VOLTS, VDIFF_MIN_VOLTS
from fsm_obj import FSM

"""
Testing file for fast mirrorcle mirror benchmarking. Allows for user to input 
several parameters to sweep mirror across an axis and calculate angular displacement
-voltage mapping. 

BE CAREFUL WITH THE MIRROR IF YOU USE IT WRONG MINUS LOTS OF MONEY (1000 i thjink)
"""

def get_frames(cam, num_frames, roi) -> tuple:

    """
    return average global centroid accross number of frames taken per location
    """
    cx = []
    cy = []
    for _ in range(num_frames):
        gray = picam.get_gray_frame(cam)
        time.sleep(0.05)
        res = centroiding.find_laser_centroid(gray, roi)
        cx.append(res[0])
        cy.append(res[1])
    
    return ((sum(cx)/len(cx)), (sum(cy)/len(cy)))

def write_to_outfile(outfile, coords, mode):
    """
    Write data from results array into outfile
    """
    with open(outfile, mode, newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["vdiffx", "vdiffy", "cx", "cy"])
        writer.writerows(coords)

# click is just an easy library for handling CLI interface --> configure parameters using cmd line
@click.command()
@click.option('-n', '--num_frames', default=5, type=int, help="Number of frames to capture per step")
@click.option('-t', '--settling_time', default=0.1, type=float, help="Settling time in between voltage steps in seconds")
@click.option('-a', '--axis', default="x", type=str, help="Axis to sweep. <x> for x-axis, <y> for y-axis <b> for both")
@click.option('-o', '--outfile', default="voltage_mapping_out.csv", type=str, help="CSV outfile name")
@click.option('-s', '--step-size', default=1, type=float, help="VDIFF step size between frames in V")
@click.option('--start', default=0, type=float, help="Start voltage in V")
@click.option('--end', default=175, type=float, help="End voltage in V")
@click.option('--resolution', default=640, help='Square resolution (e.g. 640 -> 640x480)')
@click.option('--roi', default=50, type=int, help="Size in pixels of roi around centroid")
@click.option('--mode', default="man", type=str, help="Set mode <auto> for a fast sweep <man> for stepping")
@click.option('--image_outfile', default=None, type=str, help="If set, writes grayscale image to outfile")
def cmd(num_frames, settling_time, axis, outfile, step_size, start, end, resolution, roi, mode, image_outfile):
    """
    cmd function for click to enable easy CLI
    """

    # get FSM object, picamera
    fsm = FSM()
    try:
        cam = picam.init_camera()
    except:
        input("Cam not connected. FSM is enabled, press any key to shutdown and end")
        fsm.close()
        return
    if mode == "test-cam":
        print("Taking picture with picam to test dat jit")
        gray = picam.get_gray_frame(cam)
        centroiding.grayscale_to_outfile(gray, "gray1.jpg")
        return
    # fsm now live
    active = fsm.begin()
    # reset outfile
    with open(outfile, "w", newline="") as f:
        pass
    coords = []

    if mode == "man":
        print(f'[MAN] FSM is active at {fsm.vdiff_x} Vdiff-x | {fsm.vdiff_y} Vdiff-y')
        if active != 1:
            try:
                while True:  # main loop
                    usr = input("Input vdiffx, vdiffy (x y): ")
                    lst = usr.split()
                    if len(lst) != 2:
                        print("Bad coords. Input <x y>")
                        continue
                    x, y = tuple(lst)
                    if x and y:
                        print(f'setting vdiff: x: {x}, y: {y}')
                        fsm.set_vdiff(float(x), float(y))
                        
                        centroid = get_frames(cam, num_frames, roi)
                        vdiff_x, vdiff_y = fsm.get_voltages()
                        coords.append([vdiff_x, vdiff_y, centroid[0], centroid[1]])

                    else:
                        print("Bad coords. Input <x y>")
                        continue

            except KeyboardInterrupt:
                print("Keyboard interrupt received — shutting down")
            except:
                print("Other error occurred, shutting down.")
            finally:
                fsm.close()
                write_to_outfile(outfile, coords, "a")
                return
        else:
            print("FSM failed to start up, shutting down")
            fsm.close()
            return

    elif mode == "auto":
        if active != 1:

            print(f'[AUTO] FSM is active at {fsm.vdiff_x} Vdiff-x | {fsm.vdiff_y} Vdiff-y')

            if VDIFF_MIN_VOLTS > start or start > VDIFF_MAX_VOLTS:
                print("[AUTO] Start val not in vdiff allowed range")
                fsm.close()
            if VDIFF_MIN_VOLTS > end or end > VDIFF_MAX_VOLTS:
                print("[AUTO] End val not in vdiff allowed range")
                fsm.close()

            curr_vdiff = start
            try:
                while curr_vdiff <= end:

                    if axis == "x":
                        fsm.set_vdiff(curr_vdiff, 0)
                    elif axis == "y":
                        fsm.set_vdiff(0, curr_vdiff)
                    else:
                        break

                    centroid = get_frames(cam, num_frames, roi)
                    vdiff_x, vdiff_y = fsm.get_voltages()
                    coords.append([vdiff_x, vdiff_y, centroid[0], centroid[1]])
                    
                    time.sleep(settling_time)

                    curr_vdiff += step_size

            except KeyboardInterrupt:
                print("Keyboard interrupt received — shutting down")
            except:
                print("Other error occurred, shutting down.")
            finally:
                fsm.close()
                write_to_outfile(outfile, coords, "w")
                return

    else:
        print("Invalid mode, shutting down")
        fsm.close()
        return

    # ok if everything is behaving nicely, now we need to figure out what sweep we are performing. 

if __name__ == "__main__":
    cmd()

