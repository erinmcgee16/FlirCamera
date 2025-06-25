import numpy as np
import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 18000000

def grab_frame(timeout=2.0):
    image = np.zeros((120, 160), dtype=np.uint16)

    # 240 total packets: 2 per row, 60 per segment, 4 segments
    received = np.zeros((120, 2), dtype=bool)  # [row, left/right]
    temp_rows = np.zeros((120, 2, 80), dtype=np.uint16)  # rows, halves, pixels

    start_time = time.time()

    while not np.all(received):
        if time.time() - start_time > timeout:
            print("Timeout: partial frame returned.")
            break  # prevent infinite loop

        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue  # telemetry packet

        line_number = data[1]
        segment_number = data[2]
        if not (1 <= segment_number <= 4 and 0 <= line_number < 60):
            continue

        segment_idx = segment_number - 1
        row = segment_idx * 60 + line_number
        if not (0 <= row < 120):
            continue  # extra guard

        # Distinguish left/right using MSB of data[0]
        packet_id = data[0] >> 4
        if packet_id % 2 == 0:
            half = 0  # left
        else:
            half = 1  # right

        if received[row, half]:
            continue  # already got this half

        for col in range(80):
            high = data[4 + col*2]
            low  = data[5 + col*2]
            temp_rows[row, half, col] = (high << 8) | low

        received[row, half] = True

    # Reconstruct full image
    for row in range(120):
        image[row, :80] = temp_rows[row, 0]
        image[row, 80:] = temp_rows[row, 1]

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
