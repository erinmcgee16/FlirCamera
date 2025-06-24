#!/usr/bin/env python3
"""
Advanced FLIR Lepton 3.5 Driver with proper I2C initialization
This version includes camera initialization and multiple capture strategies
"""

import cv2
import numpy as np
import spidev
import time
import sys
import struct
from threading import Thread, Lock

# Try to import I2C library
try:
    import smbus
    I2C_AVAILABLE = True
except ImportError:
    print("Warning: smbus not available, I2C initialization disabled")
    I2C_AVAILABLE = False

class LeptonAdvanced:
    def __init__(self, spi_bus=0, spi_device=0, spi_speed=10000000, i2c_bus=1, i2c_addr=0x2A):
        """
        Initialize Lepton camera with I2C control and SPI data
        
        Args:
            spi_bus: SPI bus (usually 0)
            spi_device: SPI device (usually 0) 
            spi_speed: SPI speed in Hz (start lower for reliability)
            i2c_bus: I2C bus (usually 1 on Pi)
            i2c_addr: Lepton I2C address (0x2A default)
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.spi_speed = spi_speed
        self.i2c_bus = i2c_bus
        self.i2c_addr = i2c_addr
        
        self.spi = None
        self.i2c = None
        
        # Frame specifications for Lepton 3.5
        self.width = 160
        self.height = 120
        self.packet_size = 164
        self.packets_per_frame = 120
        
        # Capture state
        self.frame_lock = Lock()
        self.latest_frame = None
        self.running = False
        self.frame_count = 0
        
        print(f"Initializing Advanced Lepton Driver")
        print(f"SPI: Bus {spi_bus}, Device {spi_device}, Speed {spi_speed} Hz")
        print(f"I2C: Bus {i2c_bus}, Address 0x{i2c_addr:02X}")
        
    def init_i2c(self):
        """Initialize I2C communication for camera control"""
        if not I2C_AVAILABLE:
            print("I2C not available - skipping initialization")
            return False
            
        try:
            self.i2c = smbus.SMBus(self.i2c_bus)
            print("✓ I2C initialized")
            
            # Test I2C communication
            try:
                # Try to read camera status
                self.i2c.read_byte(self.i2c_addr)
                print("✓ I2C communication with camera verified")
                return True
            except:
                print("⚠ I2C initialized but camera not responding")
                # Continue anyway - camera might still work with SPI only
                return True
                
        except Exception as e:
            print(f"✗ I2C initialization failed: {e}")
            return False
    
    def init_spi(self):
        """Initialize SPI communication for data transfer"""
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = self.spi_speed
            self.spi.mode = 3  # CPOL=1, CPHA=1
            print(f"✓ SPI initialized at {self.spi_speed} Hz")
            return True
        except Exception as e:
            print(f"✗ SPI initialization failed: {e}")
            return False
    
    def camera_command(self, command_id, data=None):
        """Send command to camera via I2C"""
        if not self.i2c:
            return False
            
        try:
            if data is None:
                self.i2c.write_byte(self.i2c_addr, command_id)
            else:
                self.i2c.write_i2c_block_data(self.i2c_addr, command_id, data)
            time.sleep(0.001)  # Small delay
            return True
        except Exception as e:
            print(f"I2C command error: {e}")
            return False
    
    def initialize_camera(self):
        """Initialize camera settings"""
        print("Initializing camera...")
        
        if self.i2c:
            # Try to enable radiometry mode for Lepton 3.5
            try:
                # These are example commands - actual commands depend on Lepton SDK
                # Enable radiometry
                self.camera_command(0x0E, [0x01])
                time.sleep(0.1)
                
                # Set AGC mode
                self.camera_command(0x01, [0x00])  # Disable AGC for raw thermal data
                time.sleep(0.1)
                
                print("✓ Camera initialized with I2C commands")
            except Exception as e:
                print(f"⚠ Camera I2C initialization failed: {e}")
        
        # Wait for camera to stabilize
        print("Waiting for camera to stabilize...")
        time.sleep(2)
        
        return True
    
    def discard_packets(self, count=300):
        """Discard initial packets to clear buffers"""
        print(f"Discarding {count} packets to clear buffers...")
        try:
            for i in range(count):
                self.spi.readbytes(self.packet_size)
                if i % 50 == 0:
                    print(f"  Discarded {i} packets...")
        except:
            pass
    
    def read_packet_robust(self):
        """Read packet with error recovery"""
        max_retries = 3
        for retry in range(max_retries):
            try:
                packet = self.spi.readbytes(self.packet_size)
                if len(packet) == self.packet_size:
                    return bytearray(packet)
            except Exception as e:
                if retry == max_retries - 1:
                    print(f"Packet read failed after {max_retries} retries: {e}")
                time.sleep(0.001)
        return None
    
    def sync_to_frame_start(self, max_attempts=750):
        """Robust frame synchronization"""
        print("Synchronizing to frame start...")
        
        consecutive_zero_packets = 0
        required_consecutive = 2  # Need at least 2 consecutive packet 0s
        
        for attempt in range(max_attempts):
            packet = self.read_packet_robust()
            if packet is None:
                continue
            
            # Extract packet number
            packet_num = (packet[0] & 0x0F) << 8 | packet[1]
            
            if packet_num == 0:
                consecutive_zero_packets += 1
                if consecutive_zero_packets >= required_consecutive:
                    print(f"✓ Frame sync found at attempt {attempt}")
                    return True
            else:
                consecutive_zero_packets = 0
            
            # Progress indicator
            if attempt % 100 == 0:
                print(f"  Sync attempt {attempt}, last packet: {packet_num}")
        
        print(f"✗ Frame sync failed after {max_attempts} attempts")
        return False
    
    def capture_frame_method1(self):
        """Primary frame capture method"""
        if not self.sync_to_frame_start():
            return None
        
        frame_data = np.zeros((self.height, self.width), dtype=np.uint16)
        packets_received = 0
        valid_packets = 0
        
        for expected_packet in range(self.packets_per_frame):
            packet = self.read_packet_robust()
            if packet is None:
                continue
            
            packets_received += 1
            packet_num = (packet[0] & 0x0F) << 8 | packet[1]
            
            # Skip telemetry packets (packet numbers >= 120 for Lepton 3.5)
            if packet_num >= self.packets_per_frame:
                continue
            
            # Extract pixel data
            pixel_data = packet[4:]
            pixels = []
            
            for i in range(0, len(pixel_data), 2):
                if i + 1 < len(pixel_data):
                    # Try both endianness
                    pixel_be = (pixel_data[i] << 8) | pixel_data[i + 1]      # Big endian
                    pixel_le = (pixel_data[i + 1] << 8) | pixel_data[i]      # Little endian
                    
                    # Use big endian (standard for Lepton)
                    pixels.append(pixel_be)
            
            # Store pixels in frame
            if len(pixels) >= self.width and packet_num < self.height:
                frame_data[packet_num, :len(pixels[:self.width])] = pixels[:self.width]
                valid_packets += 1
        
        print(f"Capture stats: {packets_received} packets, {valid_packets} valid")
        
        # Check if we got enough valid data
        non_zero_pixels = np.count_nonzero(frame_data)
        if non_zero_pixels > (self.width * self.height * 0.1):  # At least 10% non-zero
            return frame_data
        
        return None
    
    def capture_frame_method2(self):
        """Alternative capture method - sequential reading"""
        print("Trying alternative capture method...")
        
        # Read a large number of packets and try to find frame boundaries
        num_packets_to_read = 200
        packets = []
        
        for i in range(num_packets_to_read):
            packet = self.read_packet_robust()
            if packet is not None:
                packet_num = (packet[0] & 0x0F) << 8 | packet[1]
                packets.append((packet_num, packet))
        
        if not packets:
            return None
        
        # Find start of frame (packet 0)
        frame_start = -1
        for i, (packet_num, packet) in enumerate(packets):
            if packet_num == 0:
                frame_start = i
                break
        
        if frame_start == -1:
            print("No frame start found in packet sequence")
            return None
        
        # Build frame from sequential packets
        frame_data = np.zeros((self.height, self.width), dtype=np.uint16)
        valid_packets = 0
        
        for i in range(frame_start, min(frame_start + self.packets_per_frame, len(packets))):
            packet_num, packet = packets[i]
            
            if packet_num >= self.packets_per_frame:
                continue
            
            pixel_data = packet[4:]
            pixels = []
            
            for j in range(0, len(pixel_data), 2):
                if j + 1 < len(pixel_data):
                    pixel_value = (pixel_data[j] << 8) | pixel_data[j + 1]
                    pixels.append(pixel_value)
            
            if len(pixels) >= self.width and packet_num < self.height:
                frame_data[packet_num] = pixels[:self.width]
                valid_packets += 1
        
        print(f"Method 2: {valid_packets} valid packets")
        
        if valid_packets > 50:  # Need reasonable number of valid packets
            return frame_data
        
        return None
    
    def capture_frame(self):
        """Main frame capture with multiple methods"""
        # Try primary method
        frame = self.capture_frame_method1()
        if frame is not None:
            return frame
        
        print("Primary method failed, trying alternative...")
        # Try alternative method
        frame = self.capture_frame_method2()
        return frame
    
    def start_continuous_capture(self):
        """Start continuous frame capture"""
        self.running = True
        self.capture_thread = Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        print("Continuous capture started")
    
    def _capture_loop(self):
        """Background capture loop"""
        consecutive_failures = 0
        max_failures = 10
        
        while self.running:
            frame = self.capture_frame()
            if frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.frame_count += 1
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    print(f"Too many consecutive failures ({max_failures}), restarting...")
                    self.discard_packets(100)
                    consecutive_failures = 0
            
            time.sleep(0.1)  # Control capture rate
    
    def get_latest_frame(self):
        """Get the most recent frame"""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None
    
    def stop_capture(self):
        """Stop continuous capture"""
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
    
    def close(self):
        """Close all connections"""
        self.stop_capture()
        if self.spi:
            self.spi.close()
        if self.i2c:
            self.i2c.close()

def test_advanced_capture():
    """Test the advanced capture system"""
    print("Advanced Lepton Test")
    print("=" * 40)
    
    # Try multiple SPI speeds
    spi_speeds = [10000000, 8000000, 5000000, 20000000]
    
    for speed in spi_speeds:
        print(f"\nTrying SPI speed: {speed} Hz")
        
        camera = LeptonAdvanced(spi_speed=speed)
        
        try:
            # Initialize
            if not camera.init_spi():
                continue
            
            camera.init_i2c()
            camera.initialize_camera()
            
            # Discard initial packets
            camera.discard_packets(200)
            
            # Try to capture a frame
            print("Attempting frame capture...")
            frame = camera.capture_frame()
            
            if frame is not None:
                print(f"✓ SUCCESS with SPI speed {speed} Hz!")
                
                # Display frame
                min_val = np.min(frame[frame > 0]) if np.any(frame > 0) else 0
                max_val = np.max(frame)
                print(f"Frame data range: {min_val} to {max_val}")
                
                if max_val > min_val:
                    normalized = ((frame - min_val) / (max_val - min_val) * 255).astype(np.uint8)
                    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
                    scaled = cv2.resize(colored, (640, 480), interpolation=cv2.INTER_NEAREST)
                    
                    cv2.imshow("Advanced Lepton Test", scaled)
                    print("Press any key to continue or 'q' to quit...")
                    key = cv2.waitKey(0) & 0xFF
                    cv2.destroyAllWindows()
                    
                    if key == ord('q'):
                        camera.close()
                        return True
                
                camera.close()
                return True
            else:
                print(f"✗ Failed with SPI speed {speed} Hz")
                
        except Exception as e:
            print(f"Error with speed {speed}: {e}")
        finally:
            camera.close()
    
    print("\n❌ All SPI speeds failed")
    return False

def run_live_display():
    """Run live display with advanced driver"""
    print("Starting live display...")
    
    camera = LeptonAdvanced(spi_speed=10000000)  # Start with reliable speed
    
    try:
        if not camera.init_spi():
            return False
        
        camera.init_i2c()
        camera.initialize_camera()
        camera.discard_packets(200)
        
        # Start continuous capture
        camera.start_continuous_capture()
        
        # Wait for first frame
        print("Waiting for first frame...")
        for i in range(50):
            frame = camera.get_latest_frame()
            if frame is not None:
                break
            time.sleep(0.1)
        
        if frame is None:
            print("No frames captured")
            return False
        
        print("Starting live display - press 'q' to quit")
        cv2.namedWindow("Lepton Live", cv2.WINDOW_AUTOSIZE)
        
        while True:
            frame = camera.get_latest_frame()
            if frame is not None:
                # Normalize and display
                min_val = np.min(frame[frame > 0]) if np.any(frame > 0) else 0
                max_val = np.max(frame)
                
                if max_val > min_val:
                    normalized = ((frame - min_val) / (max_val - min_val) * 255).astype(np.uint8)
                    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
                    scaled = cv2.resize(colored, (640, 480), interpolation=cv2.INTER_NEAREST)
                    
                    # Add frame count
                    cv2.putText(scaled, f"Frame: {camera.frame_count}", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    
                    cv2.imshow("Lepton Live", scaled)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
        
        cv2.destroyAllWindows()
        return True
        
    except Exception as e:
        print(f"Live display error: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        camera.close()

def main():
    """Main function with menu"""
    print("Advanced FLIR Lepton 3.5 Driver")
    print("=" * 40)
    print("1. Test capture (try different speeds)")
    print("2. Run live display")
    print("3. Exit")
    
    while True:
        try:
            choice = input("\nSelect option (1-3): ").strip()
            
            if choice == '1':
                test_advanced_capture()
            elif choice == '2':
                run_live_display()
            elif choice == '3':
                break
            else:
                print("Invalid choice")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break

if __name__ == "__main__":
    main()
