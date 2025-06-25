import numpy as np
import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 20000000


def wait_for_frame_start():
    """Wait for the start of a new frame: first valid packet"""
    while True:
        data = spi.readbytes(164)
        if (data[0] & 0x0F) != 0x0F:
            return  # found a valid data packet

def grab_frame():
    image = np.zeros((120, 160), dtype=np.uint16)
    packets = []

    wait_for_frame_start()

    while len(packets) < 240:
        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue  # skip telemetry/discard
        packets.append(data)

    for row in range(120):
        left_packet = packets[row * 2]
        right_packet = packets[row * 2 + 1]

        for col in range(80):
            hi = left_packet[4 + col * 2]
            lo = left_packet[5 + col * 2]
            image[row, col] = (hi << 8) | lo

            hi = right_packet[4 + col * 2]
            lo = right_packet[5 + col * 2]
            image[row, col + 80] = (hi << 8) | lo

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
