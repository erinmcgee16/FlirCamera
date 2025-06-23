import numpy as np
import cv2
import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 1)  # bus 0, device CE1
spi.max_speed_hz = 18000000

def grab_frame():
    # Lepton frame is 160x120, 2 bytes per pixel, 60 packets per frame
    frame = np.zeros((60, 164), dtype=np.uint8)
    for i in range(60):
        data = spi.readbytes(164)
        if (data[0] & 0x0f) == 0x0f:
            return None  # discard segment packets
        frame[i] = np.frombuffer(bytearray(data), dtype=np.uint8)
    return frame

def extract_image(raw):
    # Skip packet headers, only take pixel data
    image = np.zeros((60, 80), dtype=np.uint16)
    for i in range(60):
        line = raw[i]
        for j in range(80):
            image[i, j] = (line[4 + j*2] << 8) | line[5 + j*2]
    return image

while True:
    raw_frame = grab_frame()
    if raw_frame is None:
        continue

    image = extract_image(raw_frame)
    norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
    disp = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_INFERNO)

    cv2.imshow("Thermal", disp)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
spi.close()
