import numpy as np
import spidev
import cv2

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 20000000

def grab_frame_fast():
    # Read data in much larger chunks to minimize timing gaps
    raw_data = []
    packets = []
    
    # Read large chunks until we have enough data
    while len(packets) < 240:
        # Read 32KB at once (about 195 packets worth of data)
        chunk = spi.readbytes(32768)
        raw_data.extend(chunk)
        
        # Process the accumulated data to extract packets
        i = 0
        while i + 164 <= len(raw_data) and len(packets) < 240:
            packet = raw_data[i:i+164]
            if (packet[0] & 0x0F) != 0x0F:  # Not a discard packet
                packets.append(packet)
            i += 164
        
        # Keep unused data for next iteration
        raw_data = raw_data[i:]
    
    # Now process all packets at once (no timing pressure)
    image = np.zeros((120, 160), dtype=np.uint16)
    
    for row in range(120):
        if row * 2 + 1 >= len(packets):
            break
            
        left_packet = packets[row * 2]
        right_packet = packets[row * 2 + 1]
        
        # Fill left half of row (pixels 0-79)
        for col in range(80):
            high = left_packet[4 + col*2]
            low  = left_packet[5 + col*2]
            image[row, col] = (high << 8) | low
            
        # Fill right half of row (pixels 80-159)  
        for col in range(80):
            high = right_packet[4 + col*2]
            low  = right_packet[5 + col*2]
            image[row, col + 80] = (high << 8) | low
    
    return image

while True:
    image = grab_frame_fast()
    norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
    disp = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_INFERNO)
    cv2.imshow("Thermal", disp)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
spi.close()
