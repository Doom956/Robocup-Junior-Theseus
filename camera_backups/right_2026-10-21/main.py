# Untitled - By: Baichen.Luo - Sat Jul 4 2026

import sensor, image, time, ml, math, uos, gc
import pyb
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=2000)
sensor.set_vflip(True)
sensor.set_hmirror(True)
uart = UART(1, 115200, timeout_char=100)
#sensor.set_auto_exposure(False, exposure_us=10000)
clock = time.clock()

FRAME_SIZE = 240
EDGE_DIST = 20          # margem de segurança das bordas
AREA_THRESHOLD = 2500
PIXELS_THRESHOLD = 300  # descarta blobs "furados"/ruidosos
DENSITY_THRESHOLD = 0.4 # fração mínima de pixels preenchidos no rect (filtra blobs irregulares)

thresholds = {
    'B': (0, 20, -128, 127, -128, 127),      # black
    'R': (0, 100, 39, 127, -128, 127),       # red
    'Y': (46, 100, -49, 39, 6, 123),         # yellow
    'G': (0, 100, -128, -3, -49, 45),        # green
    'b': (37, 100, -16, 127, -128, -30),     # blue
}

def touches_edge(b):
    return (b.x() < EDGE_DIST or
            b.y() < EDGE_DIST or
            b.x() + b.w() > FRAME_SIZE - EDGE_DIST or
            b.y() + b.h() > FRAME_SIZE - EDGE_DIST)

while True:
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8)

    detected = []
    for code, threshold in thresholds.items():
        blobs = img.find_blobs([threshold],
                                area_threshold=AREA_THRESHOLD,
                                pixels_threshold=PIXELS_THRESHOLD,
                                merge=True)
        for b in blobs:
            if touches_edge(b):
                continue
            if b.density() < DENSITY_THRESHOLD:
                continue
            img.draw_rectangle(b.rect())
            detected.append(code)
            break  # um sinal por cor por frame já basta

    for code in detected:
        uart.write(code)

    print(clock.fps())
