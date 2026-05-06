# Notes 5/5/26 - Caden, Graydon, Seth

### Systematically made characterization "tighter" by:
- Adjusting laser on mirror so there are no secondary spots on frame
- Adding thresholding to centroiding algorithm --> all pixels under threshold ignored
- Increasing centroid ROI to capture all necessary pixels (50 --> 200 pixels)
- Increasing camera resolution (640x480 --> 1280x720) (this requires recalibration for correct mapping)
- Switching setup to use raw pixel value on the naked rpi hq cam. We also used a focusing lens and turned exposure time wayyy down to make spot size on cam as small as possible
- Notably, using raw pixels values on naked sensor produced tighter residuals than using the charuco --> might ditch that method in favor of the camera sensor

### Noticed very weird sinosuidal residuals when testing x-axis behavior:
- Residuals order of magnitude more than kingsbury (reached out to him)
- His residuals only appear to oscillate very faintly, perhaps one of us is just off by some factor?
- This was consistent across every single piece of data we took

### Unresolved issues:
- X and Y axis transfer function are more different than they should be
- Sinosuidal residuals on both axes
- Possible damage to y axis on 6.4mm mirror? 

### Things to try next time:
- Further optimize camera resolution and color grading
- Get a nicer laser than the mirrorcle provided one
- Try 5mm mirror instead of 6.4mm (be more careful with this one)
- Sweep from + to - instead of - to + to see if residuals remain the same
- Use a gaussian centroiding finding method instead of current approach
- Better focusing lens? 