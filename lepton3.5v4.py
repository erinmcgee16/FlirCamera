import numpy as np
import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 18000000

def wait_for_frame_start():
    """Wait until we detect the start of a new frame: segment 1, line 0"""
    while True:
        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue  # telemetry or discard packet
        if data[1] == 0 and data[2] == 1:
            return

def grab_frame():
    image = np.zeros((120, 160), dtype=np.uint16)
    packets_needed = 240
    packets = []

    wait_for_frame_start()

    while len(packets) < packets_needed:
        data = spi.readbytes(164)
        # Filter telemetry/discard packets
        if (data[0] & 0x0F) == 0x0F:
            continue

        line_number = data[1]
        segment_number = data[2]

        # Validate line and segment
        if not (0 <= line_number < 60 and 1 <= segment_number <= 4):
            continue

        packets.append(data)

    # Stitch image from packets
    for row in range(120):
        left_packet = packets[row * 2]
        right_packet = packets[row * 2 + 1]

        for col in range(80):
            high = left_packet[4 + col*2]
            low  = left_packet[5 + col*2]
            image[row, col] = (high << 8) | low

            high = right_packet[4 + col*2]
            low  = right_packet[5 + col*2]
            image[row, col + 80] = (high << 8) | low

    return image

import cv2

while True:
    image = grab_frame()
    norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
    disp = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_INFERNO)

    cv2.imshow("Thermal", disp)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
spi.close()
