
import image, sensor, ustruct, pyb, lcd, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # use QVGA 320*240
sensor.set_auto_gain(False,gain_db = 11.4801) # must be turned off for color tracking
sensor.set_auto_whitebal(False,[60.2071,60.5557,67.1094]) # must be turned off for color tracking
lcd.init(triple_buffer=True) # Initialize the lcd screen.  Make Non-blocking but 3X RAM
msk = sensor.alloc_extra_fb(320, 210, sensor.RGB565)
msk2 = sensor.alloc_extra_fb(320, 210, sensor.RGB565)
uart = pyb.UART(3)
uart.init(115200, bits=8, parity=None)
threshold1 = (61, 100, 18, 50, 21, 42) # orange
threshold2 = (64, 100, 4, 22, -35, -7) # purple

blob_packet = '<fff'

# Setup RED LED for easier debugging
red_led   = pyb.LED(1)
green_led = pyb.LED(2)
blue_led  = pyb.LED(3)
clock = time.clock()                # Create a clock object to track the FPS.
framecount = 0
toggle = 0
offcount = 0
sensor.set_auto_exposure(False, 100000)
while True:
    clock.tick()                    # Update the FPS clock.
    if framecount % 2 == 0:
        if toggle == 0:
            blue_led.on()
            toggle = 1
            offcount = 0
        else:
            blue_led.off()
            offcount = offcount + 1
            if offcount == 20:
                toggle = 0
                offcount = 0
    img = sensor.snapshot()
    img = img.crop(roi = (0, 0, 320, 210))

    #masking and only search for ball on the ground

    #filter everything that is outside of the floor
    msk.replace(img)
    msk = msk.flood_fill(235, 209,seed_threshold=0.25,floating_threshold=0.09,invert=False,clear_background = False)
    msk = msk.flood_fill(21, 209,seed_threshold=0.25,floating_threshold=0.09,invert=False,clear_background = False)
    msk = msk.flood_fill(21, 209,seed_threshold=0.25,floating_threshold=0.09,invert=False,clear_background = True)
    msk = msk.flood_fill(134, 10,seed_threshold=0.3,floating_threshold=0.03,invert=False,clear_background = True)
   # img = img.replace(msk)

    #filter out the floor from the ball
    msk2.replace(img)
    msk2 = msk2.flood_fill(235, 209,seed_threshold=0.24,floating_threshold=0.07,invert=False,clear_background = False)
    msk2 = msk2.flood_fill(21, 209,seed_threshold=0.24,floating_threshold=0.07,invert=False,clear_background = False)
    img = img.replace(msk2)

    ###remove the out-of-floor part so only ball on the floor is detected later
    img = img.clear(msk)


    #print(sensor.get_exposure_us())
    blobs1 = img.find_blobs([threshold1], roi=(0,80,320,160), pixels_threshold=10, area_threshold=18)
    blobs2 = img.find_blobs([threshold2], roi=(0,80,320,160), pixels_threshold=10, area_threshold=18)

    if blobs1:
        blob1_sort = sorted(blobs1, key = lambda b: b.pixels(), reverse=True)
        blob1_largest = blob1_sort[:3]
        blobs1_found = len(blob1_largest)

        msg = "**".encode()
        uart.write(msg)
        for i in range(3):
            if i < blobs1_found:
                b = blob1_largest[i]
                a = float(b.area())
                x_cnt = float(b.cx())
                y_cnt = float(b.cy())
                img.draw_rectangle(b[0:4],color = [0,255,0]) # rect on x,y,w,h
                img.draw_cross(b.cx(), b.cy(),color = [0,255,0])
            else:
                a = 0.0
                x_cnt = 0.0
                y_cnt = 0.0

            # Send the blob area and centroids over UART
            b = ustruct.pack(blob_packet, a, x_cnt, y_cnt)
            uart.write(b)
    else:  # nothing found
        msg = "**".encode()
        uart.write(msg)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)

    if blobs2:
        blob2_sort = sorted(blobs2, key = lambda b: b.pixels(), reverse=True)
        blob2_largest = blob2_sort[:3]
        blobs2_found = len(blob2_largest)

        msg = "*!".encode()
        uart.write(msg)
        for i in range(3):
            if i < blobs2_found:
                b = blob2_largest[i]
                a = float(b.area())
                x_cnt = float(b.cx())
                y_cnt = float(b.cy())
                img.draw_rectangle(b[0:4],color = [0,255,0]) # rect on x,y,w,h
                img.draw_cross(b.cx(), b.cy(),color = [0,255,0])
            else:
                a = 0.0
                x_cnt = 0.0
                y_cnt = 0.0

            # Send the blob area and centroids over UART
            b = ustruct.pack(blob_packet, a, x_cnt, y_cnt)
            uart.write(b)
    else:  # nothing found
        msg = "*!".encode()
        uart.write(msg)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)
        b = ustruct.pack(blob_packet, 0.0, 0.0, 0.0)
        uart.write(b)

    lcd.display(img, roi=(96,80,128,160)) # display the image to lcd only middle 128 cols by 160 rows.
   # print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
