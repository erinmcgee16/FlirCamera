import numpy as np
import spidev
import cv2

spi = spidev.SpiDev()
spi.open(0, 1)  # bus 0, CE1
spi.max_speed_hz = 18000000

def grab_frame():
    # Prepare empty array: 120 rows × 160 columns
    image = np.zeros((120, 160), dtype=np.uint16)
    packets_per_segment = 60
    segments_collected = [False] * 4

    # Store partial packets (80 pixels per packet)
    segments = [[None]*packets_per_segment for _ in range(4)]

    while not all(segments_collected):
        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue  # discard segment sync/telemetry packets

        line_number = data[1]
        segment_number = data[2]

        if not (1 <= segment_number <= 4) or not (0 <= line_number < packets_per_segment):
            continue  # invalid packet

        # Mark this segment's line as received
        segment_idx = segment_number - 1
        segments[segment_idx][line_number] = data

        # If all 60 lines received for this segment, mark it done
        if all(segments[segment_idx]):
            segments_collected[segment_idx] = True

    # Stitch all packets into image
    for segment_idx in range(4):
        for line_number in range(packets_per_segment):
            packet = segments[segment_idx][line_number]
            row = segment_idx * 60 + line_number
            for col in range(80):
                high = packet[4 + col*2]
                low  = packet[5 + col*2]
                value = (high << 8) | low
                image[row, col] = value  # left half

    # Second pass for right half
    # Lepton 3.5 sends right half as a second packet with same line_number
    # In this simplified version, we assume sequential capture per row
    # If needed, you can add logic to distinguish left/right

    # This simplified version only fills the left 80 columns
    # To fully implement both halves, you need to track which packet is left vs right
    return image

while True:
    image = grab_frame()
    norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
    disp = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_INFERNO)

    cv2.imshow("Thermal", disp)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
spi.close()
