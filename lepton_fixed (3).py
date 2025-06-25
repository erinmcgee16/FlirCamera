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
    segment_numbers = [0, 0, 0, 0]  # To store which segment is which
    
    # Collect 240 packets and identify segments
    while len(packets) < 240:
        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue  # skip discard/telemetry packets
            
        packets.append(data)
        
        # Check if this is packet 20 (contains segment ID)
        packet_num = data[1]
        if packet_num == 20:
            # Extract segment number from ID word
            id_word = (data[0] << 8) | data[1]
            segment_num = (id_word >> 12) & 0x07
            if segment_num >= 1 and segment_num <= 4:
                segment_index = len(packets) // 60  # Which 60-packet group we're in
                if segment_index < 4:
                    segment_numbers[segment_index] = segment_num
    
    # Now map segments to correct image rows
    for segment_group in range(4):
        actual_segment = segment_numbers[segment_group]
        if actual_segment == 0:  # Invalid segment, use default mapping
            actual_segment = segment_group + 1
            
        start_packet = segment_group * 60
        start_row = (actual_segment - 1) * 30  # Segment 1->rows 0-29, 2->30-59, etc.
        
        # Process 30 rows in this segment
        for row_in_segment in range(30):
            actual_row = start_row + row_in_segment
            if actual_row >= 120:  # Safety check
                continue
                
            packet_idx = start_packet + row_in_segment * 2
            if packet_idx + 1 >= len(packets):  # Safety check
                continue
                
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