# How to use `voltage_mapping_interactive_main.py`

## Basic Info

The idea behind this script is to input certain parameters that dictate how the FSM should move, sweep the FSM in some pattern, and after each step capture the laser's position on a camera and write all relevant info to a csv so we can model the voltage-anglular displacement behavior, the repeatability, and hysterisis of the FSM. 

This script works without comand line arguments. Every time you want to use it, simply run `python3 voltage_mapping_interactive_main.py`  from the respository root. The script will then prompt you to use it in one of two ways: 

- Using a parameters json, which is the recommended use mode. Edit `voltage_mapping_params.json`to use this model See below what each field of the parameters file represents. 
- Using interactive mode, where the script will prompt you sequentially for how to run. This may be helpful for understanding but is slow.

## How to use `voltage_mapping_params.json`

Before you run the mapping script, update the `voltage_mapping_params.json` file to reflect the parameters you want for your run. Below is a description of what each field in the file does, in order of how they appear: 

### Main Parameters (Not mode specific)

- `mode:` this selects the overall mode in which the mapping script will run. Available modes are `preview_cam`, `man`, `sweep`, `sweep_raw`, `random_square_walk`, `grid`, `grid_raw`, `verify`, `verify_sweep`, and `verify_grid`. These modes essentially define the "shape" you want to run the FSM in, and mode-specific parameters are defined further down in the json. 
- `outfile:` input the path of the outfile you would like the script to write to in this field. usually we put `runs/mapping` as the directory. 
- `num_frames:` How many frames the camera captures which are then averaged over to calculate the laser centroid. usually keep it around 1-3 for faster processing
- `settling_time:` How long the script pauses inbetween moving the mirror and taking a picture. You can make this pretty low, we have noticed some "ringing" effects when settling time is super low. The current setting is fine, if you get impatient you can always lower it. At a certain point camera latency becomes dominant. 
- `resolution:` The resolution of the camera. 1280p is perfectly fine, if you want to go higher you can but may result in slower runs. 
- `roi:` "Region of Interest". The way our centrioding algorithm works by default is it takes the raw image from the camera, finds the brightest pixels, and then clips a region of size roi around those pixels, and then performs the weighted-average centroiding on that smaller square. This ensures that processing time is low, and also clips out far-off noise from affecting the centroid. 
- `no_calib:` If set to `true`, does not run charuco-based calibration on the camera. Leave this to true unless doing stuff with charuco.
- `calibration_path:` used for path to calibration params for charuco. Ignore if not using charuco. 
- `use_gaussian_centroiding` if set to `true`, on each capture the script does a more advanced centroiding algorithm. If this parameter is not enabled, default centroiding does the following:
  1. capture raw image with camera
  2. threshold all the pixels, setting pixel to zero if it is below the value `LASER_CENTROID_INTENSITY_THRESHOLD` defined in `src/constants.py`.
  3. clip the roi around the brightest pixel
  4. performed weighted-average centroiding and recalculate global pixel positions.
    If this parameter is enabled, it instead does the following more advanced, computationally expensive routine:
  1. capture raw image with camera
  2. subtract a dark, laser-free image from the captured image to eliminate noise. This image path can be specified in the config json with parameter `dark_gray_path:`.
  3. Soft threshold the pixels based on same threshold constant. This means subtract the constant from every pixel and clamp negatives to zero.
  4. Clip roi around brightest pixel
  5. attempt to fit pixel intensity distribution in that roi to a gaussian, if successful return gaussian centroid, if unsuccessful fall back to weighted-average centroiding
  6. recalculate global pixel centroid position and return
- `dark_gray_path:` input path to laser-free image if using gaussian centroiding
- `distance_to_board:` input distance to either charuco board or camera if you want to csv to also include the angular displacement calculation. This is still recoverable from raw data so this parameter is fine if inaccurrate. 
- `preview_bind`, `preview_port`, `preview_fps`, `preview_quality:` Leave these be, they just configure the preview stream from the camera that you can view in the brower for taking calibration images.

### Mode-Specific Parameters - `sweep`
This mode sweeps the laser across one axis and marks the pixel value after each step. The laser is NOT SLEWED inbetween steps. 
- `axis:` which axis the sweep should be performed on. Input `x` or `y`.
- `start_vdiff` starting vdiff of sweep.
- `end_vdiff:` ending vdiff of sweep.
- `step_size:` amount of vdiff stepped between each point. For example, starting at vdiff -150 at step size 1 would then go to -149, -148, etc. To do reverse sweeps, put the step size as negative and flip start and end vdiffs. 

### `sweep_raw`
This is the exact same as sweep, with the same parameters, but it also outputs the raw pixels values. Usually just use this one if you are working with the raw camera. 

### `random_square_walk`
This mode puts the laser in a random walk across five random points that form the corners and center of a square to test its repeatability. 
-  `diagonal_corner_a:` coordinates of bottom left of square in format `[-50.0, -50.0]`. These are in units of VDIFF. 
- `diagonal_corner_b:` coordinates of top right corner of square. 
- `step_settling_s:` optional override for default settling time. Not sure why i added this tbh its repetitive.
- `iterations:` How many random walk steps you want to do

### `grid`
Similar to sweep, but now steps the laser through a grid-like pattern from the start corner to the end corner. 
- `start_corner:` coordinates of starting corner of grid pattern in format `[-120.0, -120.0]` in units of VDIFF
- `end_corner:` coordinates of ending corner of grid pattern in units of VDIFF
- `step_size:` size between each step in grid pattern in units of VDIFF

### `grid_raw`
Exact same as `grid`, but also outputs raw pixel coordinates to the csv file. Just use this one usually if you want to do grid

### `verify`, `verify_sweep`, `verify_grid`

I kinda stopped using these, the idea is to do a sweep or grid pattern and then on the fly calculate difference bewteen the observed laser location vs the predicted location using a polynomial model, but I found its more useful to just post-process the data after. 