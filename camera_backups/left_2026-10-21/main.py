# Untitled - By: Baichen.Luo - Sat Jul 4 2026

import sensor, image, time, ml, math, uos, gc
import pyb
from pyb import UART
sensor.reset()                         # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)      # Set frame size to QVGA (320x240)
sensor.set_windowing((240, 240))       # Set 240x240 window.
sensor.skip_frames(time=2000)          # Let the camera adjust.
sensor.set_vflip(True) # the camera's upside down on the robot.
sensor.set_hmirror(True)
uart = UART(1, 115200, timeout_char=100)
sensor.set_auto_exposure(False, exposure_us=10000)
clock = time.clock()
black_threshold = (0, 20, -128, 127, -128, 127)
red_threshold = (0, 100, 39, 127, -128, 127)
yellow_threshold = (48, 100, -65, 54, -3, 123)
green_threshold = (0, 100, -128, -3, -49, 45)
blue_threshold = (37, 100, -16, 127, -128, -30)
while True:
    clock.tick()
    img = sensor.snapshot()
    black_blobs = img.find_blobs(([black_threshold]), area_threshold= 2500, merge = True);
    if black_blobs:
        for b in black_blobs:
            img.draw_rectangle(b.rect())
        uart.write('B')
    red_blobs = img.find_blobs(([red_threshold]), area_threshold= 2500, merge = True);
    if red_blobs:
        for b in red_blobs:
            img.draw_rectangle(b.rect())
        uart.write('R')
    green_blobs = img.find_blobs(([green_threshold]), area_threshold= 2500, merge = True);
    if green_blobs:
        for b in green_blobs:
            img.draw_rectangle(b.rect())
        uart.write('G')
    yellow_blobs = img.find_blobs(([yellow_threshold]), area_threshold= 2500, merge = True);
    if yellow_blobs:
        for b in yellow_blobs:
            img.draw_rectangle(b.rect())
        uart.write('Y')
    blue_blobs = img.find_blobs(([blue_threshold]), area_threshold= 2500, merge = True);
    if blue_blobs:
        for b in blue_blobs:
            img.draw_rectangle(b.rect())
        uart.write('b')
    print(clock.fps())
