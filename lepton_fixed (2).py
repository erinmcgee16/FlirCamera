import numpy as np
import spidev
import time
import cv2

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 20000000

def grab_frame_simple():
    image = np.zeros((120, 160), dtype=np.uint16)
    packets = {}  # Use dict to store packets by (segment, packet_num)
    
    # Collect packets until we have a complete frame
    while len(packets) < 240:
        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue  # skip discard/telemetry packets
            
        # Extract segment and packet number from ID word
        id_word = (data[0] << 8) | data[1]
        segment_num = (id_word >> 12) & 0x07  # Bits 14-12
        packet_num = id_word & 0x3F           # Bits 5-0
        
        # Only keep valid segments (1-4) and packet numbers (0-59)
        if segment_num >= 1 and segment_num <= 4 and packet_num < 60:
            packets[(segment_num, packet_num)] = data
    
    # Now reconstruct image using actual segment/packet info
    for (segment_num, packet_num), packet_data in packets.items():
        # Calculate which row this packet belongs to
        segment_start_row = (segment_num - 1) * 30  # Segments 1-4 -> rows 0-29, 30-59, 60-89, 90-119
        row_in_segment = packet_num // 2            # 2 packets per row
        col_offset = (packet_num % 2) * 80          # Left half (0) or right half (80)
        
        actual_row = segment_start_row + row_in_segment
        
        # Fill pixels for this packet
        for col in range(80):
            high = packet_data[4 + col*2]
            low  = packet_data[5 + col*2]
            image[actual_row, col + col_offset] = (high << 8) | low
    
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