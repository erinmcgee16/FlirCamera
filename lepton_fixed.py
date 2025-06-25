import numpy as np
import spidev
import time
import cv2

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 20000000

def grab_frame_simple():
    image = np.zeros((120, 160), dtype=np.uint16)
    packets = []
    
    # Collect 240 packets
    while len(packets) < 240:
        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue  # skip discard/telemetry packets
        packets.append(data)
    
    # Process each segment
    for segment in range(4):
        start_packet = segment * 60  # Each segment has 60 packets
        start_row = segment * 30     # Each segment covers 30 rows
        
        # Process 30 rows in this segment
        for row_in_segment in range(30):
            actual_row = start_row + row_in_segment
            packet_idx = start_packet + row_in_segment * 2
            
            left_packet = packets[packet_idx]
            right_packet = packets[packet_idx + 1]
            
            # Fill left half of row (pixels 0-79)
            for col in range(80):
                high = left_packet[4 + col*2]
                low  = left_packet[5 + col*2]
                image[actual_row, col] = (high << 8) | low
                
            # Fill right half of row (pixels 80-159)  
            for col in range(80):
                high = right_packet[4 + col*2]
                low  = right_packet[5 + col*2]
                image[actual_row, col + 80] = (high << 8) | low
    
    return image

while True:
    image = grab_frame_simple()
    norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
    disp = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_INFERNO)
    cv2.imshow("Thermal", disp)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
spi.close()