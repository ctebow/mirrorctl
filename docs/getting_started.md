# Getting Started - READ THIS FIRST

Follow these instructions to set up the FSM, and make sure to check out each main scripts individual info file for commands and in depth explanations. 

## Connecting to the raspberry pi + power up

Our current setup has a raspberry pi 4b connected to mirrorcle's digital-input driver, which is connected to the FSM. The raspberry pi is connected to a breakout pcb with the colorful jumper wires that are taped together. The breakout pcb then connects to the driver with a ribbon cable, and the driver connects to the fsm with another ribbon cable. See mirrorcles driver documentation for which side of the driver connects to the mirror. Here is how to power things on. Always do things in this order: 

1. Before connecting any power, verify that the raspberry pi, breakout pcb, driver, and fsm plus any of their cables have no bare metal touching the optical table. The bottoms of the pcb and driver should be taped, but always double check.
2. Ensure the driver is plugged in to the fsm. Then, power on the raspberry pi by plugging in the usb-c power cable. Raspberry pi will take a minute to boot up. On the driver, there are some lights that are partially covered by tape. The green light should be on, which indicates power. The blue light may or may not be on, it doesn't really matter (this light indicates the pi is sending a clock to it over pwm). The red light should not be on until the mirror is in use.
3. Once everything is powered on, to control the raspberry pi you will need to connect to it via ssh with your computer. The easiest way to do this, and the way we have set up in lab, is to make sure the pi is connected to the router that is in lab via an ethernet cable. Then, connect your computer to the same router via ethernet. The router should already be working if you plug it in. If not, its information is on a sticker on the bottom. An added bonus of the router is you will now get wifi in lab yessirrrr.
4. Once everything is physically connected, there are two ways to connect over ssh:

- Option One: In your terminal on the computer, use the command `ssh ctebow@192.168.8.153`. You'll now have access to the raspberry pi with a command line interface. The password is `ctebow`. This works on windows and mac computers.
- Option Two: Use the ssh extension in VS code to connect to a host, then just type in the same host name: `ctebow@192.168.8.153` (password `ctebow`) This is recommended cause you can use an agent to code stuff easily and also edit the code in a visual editor.

You can also easily access the router information and see which devices are connected once you are connected to it by going to a web brower on your computer and visiting the url: `http://192.168.8.1/` and using the password `caden`. 

1. Once you are inside the raspberry pi, run the command `sudo pigpiod`. This exposes the gpio pins for the python scripts to use.
2. To go to the source code (that contains this repository!) navitage to the pulseA_fsm folder, and then eventually find the `mirrorctl` folder. This is the top level directory you will want to work in and run all commands from.
3. From there you are good to go! Just make sure to be safe with the mirror. Also ensure you are committing and pushing all changes to github, the raspberry pi is linked to my git repository and is currently on a development branch. Merge conflicts will happen so be prepared.

## Configuring the code for mirror type

All of the baseline parameters for the mirror are in `src/constants.py` in the `mirrorctl` repository. THE TWO ABSOLUTE MOST IMPORTANT ARE `VDIFF_MAX/MIN` AND `FCLK_HZ`. See the mirror datasheet for the appropropriate configurations of these constants, but their recommended values are left in comments in the constants.py file. I would recommend keeping `VDIFF_MAX` and `VDIFF_MIN` at 170V and -170V pretty much always, and only updating `FLCK_HZ` any time you switch the mirror type. Honestly leave all the other constants alone unless you are confident in the changes, bad constants can and will break the mirror. If you have the right constants it is pretty hard to do damage however. 

The rest of the parameters in that file are the pins used on the raspberry pi, setting the correct spi mode, commands to send to the DAC, and setup parameters for the camera. The only other parameters you may want to change are the camera ones which are at the bottom and pretty self-explanatory. 

## Setting up the camera

The camera should already be plugged in to the raspberry pi, if not look up how to do it online (how to plug in raspberry. pi camera to rpi 4b). Once plugged in, its use is very simple, each script that uses it runs its own setup routine that either references camera parameters (exposure time, focus, stuff like that) that the user inputs, or defaults to the ones in `src/constants.py`. The camera currently does not have its lens attatched, the lens should be somewhere around on the table nearby if you want to use it. A super super helpful script is `config/preview_stream.py`. Run this script and then copy-paste the url is prints to terminal to get a preview stream of what the camera is seeing. 

## Controlling the FSM

You can control the FSM to do various things using the three following scripts: 

- `go_to_voltage_main.py` -> This is a great test script. All it does it allow you to move the FSM to various VDIFF X, VDIFF Y combinations. Simply run the script with no arguments to control how the mirror moves. Notice that once you input Y, the red light on the mirrorcle DAC lights up to indicate the FSM is enabled. 
- `voltage_mapping_interactive_main.py` -> This is the comprehensive mapping script. Use it in combination with the camera to get voltage - angular displacement mappings for the FSM in a variety of different ways. See `docs/mapping.md` on how to use it. 
- `pid_mock_main.py` -> This is our main pid testing script. It puts the FSM in a closed loop with the camera, and uses raw pixel centroid values as the error from the setpoint of 0, 0 on the camera. The great thing about this script is you can use the second fsm to inject pointing error into the pid fsm (me and gradyon did this for his thesis). To see how to use this script, look at `docs/pid_mock_main.md` .

## About the `config/` folder

This folder is semi-outdated. We originally used charuco boards for getting voltage mapping data but figured out using the raw camera sensor is more accurate, which is what we are currently using. This folder's main purpose is to provide scripts to help capture images for creating lens undistortion and homography parameters, which you DO NOT NEED TO DO IF YOU ARE USING JUST THE RAW CAMERA SENSOR. You will probably only use the preview_stream.py file that is in this repo. You can see in some of the other scripts that there are options that say `no_calib` or something similar, which is completely ok to leave as true if you are using the raw camera sensor with no lens. 