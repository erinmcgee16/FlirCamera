#!/usr/bin/env python3
"""
FLIR Lepton 3.5 Debug Tool
This version provides detailed debugging information to help identify connection issues
"""

import cv2
import numpy as np
import spidev
import time
import sys
import struct

class LeptonDebug:
    def __init__(self, spi_bus=0, spi_device=0, spi_speed=20000000):
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.spi_speed = spi_speed
        self.spi = None
        
        # Lepton 3.5 specifications
        self.width = 160
        self.height = 120
        self.packet_size = 164
        self.packets_per_frame = 120
        
        print(f"Initializing Lepton Debug Tool")
        print(f"Expected frame size: {self.width}x{self.height}")
        print(f"Packet size: {self.packet_size} bytes")
        print(f"Packets per frame: {self.packets_per_frame}")
        
    def init_spi(self):
        """Initialize SPI with detailed error reporting"""
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = self.spi_speed
            self.spi.mode = 3  # CPOL=1, CPHA=1
            print(f"✓ SPI initialized successfully")
            print(f"  Bus: {self.spi_bus}, Device: {self.spi_device}")
            print(f"  Speed: {self.spi_speed} Hz")
            print(f"  Mode: {self.spi.mode}")
            return True
        except Exception as e:
            print(f"✗ SPI initialization failed: {e}")
            return False
    
    def test_spi_communication(self):
        """Test basic SPI communication"""
        print("\nTesting SPI communication...")
        try:
            # Try to read some data
            test_data = self.spi.readbytes(10)
            print(f"✓ SPI read test successful")
            print(f"  Sample data: {[hex(b) for b in test_data[:10]]}")
            return True
        except Exception as e:
            print(f"✗ SPI read test failed: {e}")
            return False
    
    def analyze_packets(self, num_packets=10):
        """Analyze packet structure"""
        print(f"\nAnalyzing {num_packets} packets...")
        valid_packets = 0
        packet_numbers = []
        
        for i in range(num_packets):
            try:
                packet = self.spi.readbytes(self.packet_size)
                if len(packet) == self.packet_size:
                    # Extract packet number from header
                    packet_num = (packet[0] & 0x0F) << 8 | packet[1]
                    packet_numbers.append(packet_num)
                    
                    if i < 3:  # Show details for first few packets
                        print(f"  Packet {i}: ID={packet_num}, Header={[hex(b) for b in packet[:4]]}")
                    
                    # Check if this looks like a valid image packet
                    if packet_num < 200:  # Reasonable packet number
                        valid_packets += 1
                else:
                    print(f"  Packet {i}: Wrong size ({len(packet)} bytes)")
            except Exception as e:
                print(f"  Packet {i}: Read error - {e}")
        
        print(f"Valid packets: {valid_packets}/{num_packets}")
        if packet_numbers:
            print(f"Packet number range: {min(packet_numbers)} to {max(packet_numbers)}")
        
        return valid_packets > 0
    
    def find_frame_sync(self, max_attempts=500):
        """Try to find frame synchronization"""
        print(f"\nSearching for frame sync (max {max_attempts} attempts)...")
        
        for attempt in range(max_attempts):
            try:
                packet = self.spi.readbytes(self.packet_size)
                if len(packet) >= 4:
                    packet_num = (packet[0] & 0x0F) << 8 | packet[1]
                    
                    if packet_num == 0:
                        print(f"✓ Found frame start at attempt {attempt}")
                        print(f"  Header: {[hex(b) for b in packet[:8]]}")
                        return True
                    
                    if attempt < 10 or attempt % 50 == 0:
                        print(f"  Attempt {attempt}: packet_num={packet_num}")
                        
            except Exception as e:
                if attempt < 5:
                    print(f"  Attempt {attempt}: Error - {e}")
        
        print(f"✗ Could not find frame sync after {max_attempts} attempts")
        return False
    
    def capture_test_frame(self):
        """Attempt to capture a single frame with debugging"""
        print("\nAttempting to capture test frame...")
        
        # Try to sync to frame start
        if not self.find_frame_sync():
            return None
        
        frame_data = np.zeros((self.height, self.width), dtype=np.uint16)
        valid_rows = 0
        packet_errors = 0
        
        print("Capturing frame data...")
        for row in range(self.packets_per_frame):
            try:
                packet = self.spi.readbytes(self.packet_size)
                if len(packet) != self.packet_size:
                    packet_errors += 1
                    continue
                
                packet_num = (packet[0] & 0x0F) << 8 | packet[1]
                
                # Skip telemetry packets
                if packet_num >= self.packets_per_frame:
                    continue
                
                # Extract pixel data
                pixel_data = packet[4:]
                pixels = []
                for i in range(0, len(pixel_data), 2):
                    if i + 1 < len(pixel_data):
                        pixel_value = (pixel_data[i] << 8) | pixel_data[i + 1]
                        pixels.append(pixel_value)
                
                if len(pixels) >= self.width and packet_num < self.height:
                    frame_data[packet_num] = pixels[:self.width]
                    valid_rows += 1
                
                if row < 5 or row % 20 == 0:
                    print(f"  Row {row}: packet_num={packet_num}, pixels={len(pixels)}")
                    
            except Exception as e:
                packet_errors += 1
                if packet_errors < 3:
                    print(f"  Row {row}: Error - {e}")
        
        print(f"Frame capture complete:")
        print(f"  Valid rows: {valid_rows}/{self.packets_per_frame}")
        print(f"  Packet errors: {packet_errors}")
        
        if valid_rows > 0:
            min_val = np.min(frame_data[frame_data > 0])
            max_val = np.max(frame_data)
            non_zero = np.count_nonzero(frame_data)
            print(f"  Data range: {min_val} to {max_val}")
            print(f"  Non-zero pixels: {non_zero}/{self.width * self.height}")
            
            return frame_data if non_zero > 100 else None
        
        return None
    
    def run_diagnostics(self):
        """Run complete diagnostic sequence"""
        print("FLIR Lepton 3.5 Diagnostic Tool")
        print("=" * 50)
        
        # Test 1: SPI initialization
        if not self.init_spi():
            print("\n❌ CRITICAL: SPI initialization failed")
            print("Check connections and permissions")
            return False
        
        # Test 2: Basic communication
        if not self.test_spi_communication():
            print("\n❌ CRITICAL: SPI communication failed")
            print("Check physical connections")
            return False
        
        # Test 3: Packet analysis
        if not self.analyze_packets():
            print("\n⚠️  WARNING: No valid packets detected")
            print("Camera may not be connected or powered")
        
        # Test 4: Frame capture
        frame = self.capture_test_frame()
        
        if frame is not None:
            print("\n✓ SUCCESS: Frame captured successfully!")
            
            # Display the frame
            normalized = ((frame - np.min(frame)) / (np.max(frame) - np.min(frame)) * 255).astype(np.uint8)
            colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
            scaled = cv2.resize(colored, (640, 480), interpolation=cv2.INTER_NEAREST)
            
            cv2.imshow("Lepton Test Frame", scaled)
            print("Press any key to close the test window...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            return True
        else:
            print("\n❌ FAILED: Could not capture valid frame")
            print("Possible issues:")
            print("- Camera not connected properly")
            print("- Power supply issues")
            print("- Wrong SPI settings")
            print("- Hardware failure")
            return False
    
    def close(self):
        """Clean up resources"""
        if self.spi:
            self.spi.close()

def main():
    """Main diagnostic function"""
    debug_tool = LeptonDebug()
    
    try:
        success = debug_tool.run_diagnostics()
        
        if success:
            print("\n" + "=" * 50)
            print("✓ DIAGNOSIS: Camera appears to be working!")
            print("You can now try running the main program:")
            print("python3 lepton_live_video.py")
        else:
            print("\n" + "=" * 50)
            print("❌ DIAGNOSIS: Issues detected")
            print("Please check:")
            print("1. Physical connections (see wiring diagram)")
            print("2. Power supply (3.3V)")
            print("3. SPI/I2C enabled in raspi-config")
            print("4. User permissions (groups: spi, i2c, gpio)")
            
    except KeyboardInterrupt:
        print("\nDiagnostics interrupted by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        debug_tool.close()

if __name__ == "__main__":
    main()
