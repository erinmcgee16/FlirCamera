import numpy as np
import spidev
import cv2

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 20000000

def grab_frame_sync():
    image = np.zeros((120, 160), dtype=np.uint16)
    
    # Step 1: Find start of segment 1 by looking for packet 20 with segment 1
    print("Syncing to frame start...")
    while True:
        data = spi.readbytes(164)
        if (data[0] & 0x0F) == 0x0F:
            continue
            
        packet_num = data[1]
        if packet_num == 20:
            id_word = (data[0] << 8) | data[1]
            segment_num = (id_word >> 12) & 0x07
            if segment_num == 1:
                print("Found segment 1!")
                break
    
    # Step 2: Read backwards to find packet 0 of segment 1
    # Since we found packet 20, we need to read 20 more packets to get to start of segment 2
    for _ in range(20):
        data = spi.readbytes(164)
        if (data[0] & 0x0F) != 0x0F:
            continue
    
    # Step 3: Now we should be at start of segment 2, collect all remaining packets fast
    packets = []
    target = 180  # Segments 2,3,4 = 180 packets
    
    while len(packets) < target:
        data = spi.readbytes(164)
        if (data[0] & 0x0F) != 0x0F:
            packets.append(data)
    
    print(f"Collected {len(packets)} packets")
    
    # Step 4: Process packets (segments 2,3,4 map to rows 30-119)
    for i in range(0, min(len(packets), 180), 2):
        if i + 1 >= len(packets):
            break
            
        row = 30 + i // 2  # Start from row 30 (segment 2)
        if row >= 120:
            break
            
        left_packet = packets[i]
        right_packet = packets[i + 1]
        
        # Fill left half
        for col in range(80):
            high = left_packet[4 + col*2]
            low  = left_packet[5 + col*2]
            image[row, col] = (high << 8) | low
            
        # Fill right half
        for col in range(80):
            high = right_packet[4 + col*2]
            low  = right_packet[5 + col*2]
            image[row, col + 80] = (high << 8) | low
    
    return image

# Test just a few frames to see sync results
for i in range(5):
    print(f"\n--- Frame {i+1} ---")
    image = grab_frame_sync()
    norm = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
    disp = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_INFERNO)
    cv2.imshow("Thermal", disp)
    cv2.waitKey(1000)  # Show each frame for 1 second

cv2.destroyAllWindows()
spi.close()
