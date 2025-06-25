import numpy as np
import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 20000000

def grab_frame(timeout=2.0):
    image = np.zeros((120, 160), dtype=np.uint16)
    received = np.zeros((120, 2), dtype=bool)  # rows × [left, right]
    temp = np.zeros((120, 2, 80), dtype=np.uint16)  # rows × halves × pixels

    start = time.time()

    while not np.all(received):
        if time.time() - start > timeout:
            print("Timeout — returning partial frame")
            break

        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue  # telemetry or discard packet

        line_number = data[1]
        segment_number = data[2]
        if not (0 <= line_number < 60 and 1 <= segment_number <= 4):
            continue

        row = (segment_number - 1) * 60 + line_number
        if not (0 <= row < 120):
            continue

        # Detect left/right half
        packet_id = data[0] >> 4
        half = 0 if packet_id % 2 == 0 else 1  # even = left, odd = right

        if received[row, half]:
            continue  # already have this half

        for col in range(80):
            hi = data[4 + col * 2]
            lo = data[5 + col * 2]
            temp[row, half, col] = (hi << 8) | lo

        received[row, half] = True

    # Stitch image together
    for row in range(120):
        image[row, :80] = temp[row, 0]  # left
        image[row, 80:] = temp[row, 1]  # right

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
