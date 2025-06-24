import numpy as np
import cv2
import spidev
import time

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 1)  # bus 0, device CE1
spi.max_speed_hz = 18000000

# Lepton 3.5 constants
PACKET_SIZE = 164
PACKETS_PER_SEGMENT = 60
SEGMENTS_PER_FRAME = 4
FRAME_WIDTH = 160
FRAME_HEIGHT = 120

def grab_segment():
    """Grab a single segment (60 packets) from Lepton 3.5"""
    max_attempts = 750
    
    for attempt in range(max_attempts):
        packets = []
        
        # Look for segment start (packet 0)
        while True:
            data = spi.readbytes(PACKET_SIZE)
            packet_number = data[1]
            
            # Check if this is packet 0 (segment start)
            if packet_number == 0:
                # Check if it's a discard packet
                if (data[0] & 0x0f) == 0x0f:
                    return None  # Discard segment
                packets.append(data)
                break
        
        # Read remaining packets (1-59) for this segment
        for expected_packet in range(1, PACKETS_PER_SEGMENT):
            data = spi.readbytes(PACKET_SIZE)
            packet_number = data[1]
            
            # Check for discard packet
            if (data[0] & 0x0f) == 0x0f:
                return None  # Discard segment
                
            # Check if packet number matches expected
            if packet_number != expected_packet:
                packets = []  # Reset and start over
                break
                
            packets.append(data)
        
        # If we successfully read all 60 packets
        if len(packets) == PACKETS_PER_SEGMENT:
            return packets
    
    return None

def grab_frame():
    """Grab a complete frame (4 segments) from Lepton 3.5"""
    segments = []
    
    for segment_num in range(SEGMENTS_PER_FRAME):
        segment = grab_segment()
        if segment is None:
            # If we get a discard segment, restart the frame
            segments = []
            segment_num = -1  # Will be incremented to 0
            continue
        segments.append(segment)
    
    return segments if len(segments) == SEGMENTS_PER_FRAME else None

def extract_image(segments):
    """Extract 160x120 thermal image from 4 segments of packet data"""
    if segments is None or len(segments) != SEGMENTS_PER_FRAME:
        return None
    
    # Initialize image array
    image = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype=np.uint16)
    
    for segment_idx, segment in enumerate(segments):
        if segment is None or len(segment) != PACKETS_PER_SEGMENT:
            continue
            
        for packet_idx, packet in enumerate(segment):
            # Skip the 4-byte header, extract pixel data
            pixel_data = packet[4:]
            
            # Convert bytes to uint16 values (big endian)
            line_data = np.frombuffer(bytearray(pixel_data), dtype='>u2')
            
            # Calculate the row in the final image
            # Each segment covers 30 rows (120 total rows / 4 segments)
            # Each packet in a segment covers 1 row
            row = segment_idx * 30 + packet_idx
            
            if row < FRAME_HEIGHT and len(line_data) >= 80:
                # Each packet contains 80 pixels (one row of 160 pixels is split across 2 packets)
                # But for Lepton 3.5, each packet actually contains a full row of 80 pixels
                # The 160x120 image is achieved through the 4 segments
                image[row, :80] = line_data[:80]
    
    return image

def apply_radiometric_calibration(image):
    """Apply basic radiometric calibration to improve image quality"""
    if image is None:
        return None
    
    # Remove clearly invalid pixels
    valid_mask = (image > 0) & (image < 16383)  # 14-bit range
    
    if np.sum(valid_mask) == 0:
        return image
    
    # Calculate statistics on valid pixels only
    valid_pixels = image[valid_mask]
    if len(valid_pixels) == 0:
        return image
        
    mean_val = np.mean(valid_pixels)
    std_val = np.std(valid_pixels)
    
    # Clip outliers to improve contrast
    lower_bound = np.percentile(valid_pixels, 1)
    upper_bound = np.percentile(valid_pixels, 99)
    
    clipped = np.clip(image, lower_bound, upper_bound)
    
    return clipped

def flat_field_correction(image):
    """Apply basic flat field correction to reduce fixed pattern noise"""
    if image is None:
        return None
    
    # Simple flat field correction using image statistics
    # In practice, you'd use a calibration frame
    mean_val = np.mean(image)
    corrected = image.astype(np.float32)
    
    # Subtract row-wise mean to reduce horizontal striping
    for row in range(image.shape[0]):
        row_mean = np.mean(corrected[row, :])
        if row_mean > 0:
            corrected[row, :] = corrected[row, :] - row_mean + mean_val
    
    return np.clip(corrected, 0, 65535).astype(np.uint16)

print("Starting FLIR Lepton 3.5 capture...")
print("Camera sends data in 4 segments of 60 packets each")
print("Press ESC to exit, 's' to save frame")

frame_count = 0
start_time = time.time()
successful_frames = 0

try:
    while True:
        # Grab complete frame (4 segments)
        segments = grab_frame()
        if segments is None:
            print("Failed to grab complete frame, retrying...")
            continue
        
        # Extract image from segments
        image = extract_image(segments)
        if image is None:
            continue
        
        successful_frames += 1
        
        # Apply corrections
        corrected = flat_field_correction(image)
        calibrated = apply_radiometric_calibration(corrected)
        
        if calibrated is None:
            continue
        
        # Normalize for display
        norm = cv2.normalize(calibrated, None, 0, 255, cv2.NORM_MINMAX)
        
        # Apply colormap
        display_image = cv2.applyColorMap(norm.astype(np.uint8), cv2.COLORMAP_INFERNO)
        
        # Resize for better viewing
        display_image = cv2.resize(display_image, (640, 480), interpolation=cv2.INTER_CUBIC)
        
        # Add frame info
        cv2.putText(display_image, f"Frame: {successful_frames}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Show frame
        cv2.imshow("FLIR Lepton 3.5 Thermal (160x120)", display_image)
        
        # Calculate and display frame rate
        frame_count += 1
        if frame_count % 30 == 0:
            elapsed = time.time() - start_time
            fps = successful_frames / elapsed
            print(f"Successful frames: {successful_frames}, FPS: {fps:.1f}")
        
        # Check for exit
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            break
        elif key == ord('s'):  # Save current frame
            # Save both raw and processed versions
            cv2.imwrite(f"thermal_processed_{int(time.time())}.png", display_image)
            cv2.imwrite(f"thermal_raw_{int(time.time())}.png", norm)
            print("Frames saved")

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    cv2.destroyAllWindows()
    spi.close()
    elapsed = time.time() - start_time
    if elapsed > 0:
        final_fps = successful_frames / elapsed
        print(f"Final stats - Successful frames: {successful_frames}, Average FPS: {final_fps:.1f}")
    print("Cleanup complete")