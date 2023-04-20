# Untitled - By: prenk - Fri Jan 21 2022

import sensor, image, time
import pyb, ustruct

# Define Threshold for LAB Channels
L_min = 0
L_max = 100

A_min = 31
A_max = 66

B_min = 29
B_max = 76

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 320 columns by 240 rows
sensor.skip_frames(time = 2000) # Skip two seconds worth of frames
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

# Make clock object to measure frames per second
clock = time.clock()

# Get the width and height of the image in pixels
W = sensor.width()
H = sensor.height()

while(True):
    clock.tick() # Used to calculate FPS
    img = sensor.snapshot() # Store image in frame buffer, return image object as img

    #COMPLETE: Initialize worker variables used to find centroid of pixels in the threshold

    # To initialize a variable in Python, simply type the name of the variable and set it equal to a value
    # E.X.: x = 5

    # Name one of the variables "pixel_cnt" which keeps tracks of the total number of pixels in the threshold

    #---------
    pixel_cnt = 0
    sumCol = 0
    sumRow = 0

    #---------

    # Two for loops perform a full raster scan over the entire 320x240 pixel image, one pixel at a time.
    for col in range(W):
        for row in range(H):
            pixel_rgb = img.get_pixel(col, row, rgb_tuple=True) # Get RGB color of pixel
            pixel_lab = image.rgb_to_lab(pixel_rgb) # Convert RGB color to LAB space

            # Assign L,A,B channels in pixel_lab array to separate variables
            pixel_l = pixel_lab[0]
            pixel_a = pixel_lab[1]
            pixel_b = pixel_lab[2]

            # Check whether the pixel is in the threshold
            if (L_min < pixel_l < L_max) and (A_min < pixel_a < A_max) and (B_min < pixel_b < B_max): # All channels are in their thresholds

                # COMPLETE: Incrementing Counters
                # Use worker variables to perform steps required to calculate centroid of all pixels in threshold
                #---------
                pixel_cnt = pixel_cnt + 1
                sumCol = sumCol + col
                sumRow = sumRow + row
                #---------

                # Color all pixels in threshold black (RGB argument)
                img.set_pixel(col,row,(0,0,0))

    if pixel_cnt > 0:
        # COMPLETE: Calculate Centroid
        # Calculate the centroid of the detected pixels:
        #---------

        x_cent = int(sumCol/pixel_cnt)
        y_cent = int(sumRow/pixel_cnt)

        #---------

        # Draw a cross at the centroid
        img.draw_cross(x_cent, y_cent)

        # Print centroid to the terminal
        print(f"Centroid at: {x_cent}, {y_cent}")

    else:
        x_cent = 0
        y_cent = 0
        print("Blob not found")

    # Print FPS to the serial terminal
    print("FPS: ", clock.fps())

