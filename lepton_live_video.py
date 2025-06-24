#!/usr/bin/env python3
"""
FLIR Lepton 3.5 Live Video Display for Raspberry Pi 5
Modern replacement for the Qt4-based code using OpenCV and Python

Requirements:
- FLIR Lepton 3.5 thermal camera
- Lepton Breakout Board v2
- Raspberry Pi 5 with SPI and I2C enabled
- Python libraries: opencv-python, numpy, spidev

Hardware connections for Breakout Board v2:
- VIN -> 3.3V (Pin 1)
- GND -> Ground (Pin 6)
- SDA -> GPIO 2 (Pin 3) - I2C Data
- SCL -> GPIO 3 (Pin 5) - I2C Clock
- CS -> GPIO 8 (Pin 24) - SPI Chip Select
- CLK -> GPIO 11 (Pin 23) - SPI Clock
- MISO -> GPIO 9 (Pin 21) - SPI Data Input
- MOSI -> GPIO 10 (Pin 19) - SPI Data Output (not used but connect for completeness)

Setup Instructions:
1. Enable SPI and I2C:
   sudo raspi-config -> Interface Options -> SPI -> Enable
   sudo raspi-config -> Interface Options -> I2C -> Enable

2. Install required packages:
   sudo apt update
   sudo apt install python3-opencv python3-numpy python3-pip
   pip3 install spidev

3. Run the script:
   python3 lepton_live_video.py
"""

import cv2
import numpy as np
import spidev
import time
import sys
import signal
from threading import Thread, Lock
import struct

class LeptonCamera:
    def __init__(self, spi_bus=0, spi_device=0, spi_speed=20000000):
        """
        Initialize the Lepton camera interface
        
        Args:
            spi_bus: SPI bus number (usually 0)
            spi_device: SPI device number (usually 0)
            spi_speed: SPI clock speed in Hz
        """
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.spi_speed = spi_speed
        self.spi = None
        self.frame_lock = Lock()
        self.latest_frame = None
        self.running = False
        
        # Lepton 3.5 specifications
        self.width = 160
        self.height = 120
        self.packet_size = 164  # 160 pixels * 2 bytes + 4 byte header
        self.packets_per_frame = 120
        
        # Initialize SPI
        self._init_spi()
        
    def _init_spi(self):
        """Initialize SPI connection"""
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = self.spi_speed
            self.spi.mode = 3  # CPOL=1, CPHA=1
            print(f"SPI initialized: Bus {self.spi_bus}, Device {self.spi_device}, Speed {self.spi_speed} Hz")
        except Exception as e:
            print(f"Failed to initialize SPI: {e}")
            sys.exit(1)
    
    def _read_packet(self):
        """Read a single packet from the Lepton"""
        try:
            packet = self.spi.readbytes(self.packet_size)
            return bytearray(packet)
        except Exception as e:
            print(f"Error reading packet: {e}")
            return None
    
    def _sync_frame(self):
        """Synchronize to the start of a frame"""
        max_sync_attempts = 300
        for _ in range(max_sync_attempts):
            packet = self._read_packet()
            if packet is None:
                continue
                
            # Check if this is a valid packet (packet number in first 2 bytes)
            packet_num = (packet[0] & 0x0F) << 8 | packet[1]
            if packet_num == 0:
                return True
        return False
    
    def _capture_frame(self):
        """Capture a single thermal frame"""
        # Synchronize to frame start
        if not self._sync_frame():
            return None
        
        # Initialize frame buffer
        frame_data = np.zeros((self.height, self.width), dtype=np.uint16)
        
        # Read all packets for the frame
        for row in range(self.packets_per_frame):
            packet = self._read_packet()
            if packet is None:
                return None
            
            # Extract packet number
            packet_num = (packet[0] & 0x0F) << 8 | packet[1]
            
            # Skip non-image packets (like telemetry)
            if packet_num >= self.packets_per_frame:
                continue
            
            # Extract pixel data (skip 4-byte header)
            pixel_data = packet[4:]
            
            # Convert bytes to 16-bit values (big-endian)
            pixels = []
            for i in range(0, len(pixel_data), 2):
                if i + 1 < len(pixel_data):
                    pixel_value = (pixel_data[i] << 8) | pixel_data[i + 1]
                    pixels.append(pixel_value)
            
            # Store in frame buffer
            if len(pixels) == self.width and packet_num < self.height:
                frame_data[packet_num] = pixels[:self.width]
        
        return frame_data
    
    def _frame_capture_thread(self):
        """Background thread for continuous frame capture"""
        while self.running:
            frame = self._capture_frame()
            if frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
            time.sleep(0.01)  # Small delay to prevent overwhelming the system
    
    def start_capture(self):
        """Start continuous frame capture in background thread"""
        self.running = True
        self.capture_thread = Thread(target=self._frame_capture_thread)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        print("Frame capture started")
    
    def stop_capture(self):
        """Stop frame capture"""
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        print("Frame capture stopped")
    
    def get_latest_frame(self):
        """Get the most recent frame"""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None
    
    def close(self):
        """Close the SPI connection"""
        self.stop_capture()
        if self.spi:
            self.spi.close()
        print("Lepton camera closed")

class ThermalDisplay:
    def __init__(self, camera):
        """
        Initialize the thermal display
        
        Args:
            camera: LeptonCamera instance
        """
        self.camera = camera
        self.window_name = "FLIR Lepton 3.5 Thermal Camera"
        self.display_scale = 4  # Scale factor for display
        
        # Color mapping settings
        self.colormap = cv2.COLORMAP_JET  # Default colormap
        self.auto_scale = True
        self.manual_min = 0
        self.manual_max = 65535
        
        # Statistics
        self.frame_count = 0
        self.fps_start_time = time.time()
        
    def _normalize_frame(self, frame):
        """Normalize frame data for display"""
        if frame is None:
            return None
        
        # Convert to float for processing
        frame_float = frame.astype(np.float32)
        
        if self.auto_scale:
            # Auto-scale based on current frame
            min_val = np.min(frame_float)
            max_val = np.max(frame_float)
            if max_val > min_val:
                normalized = (frame_float - min_val) / (max_val - min_val) * 255
            else:
                normalized = np.zeros_like(frame_float)
        else:
            # Use manual scaling
            normalized = np.clip(
                (frame_float - self.manual_min) / (self.manual_max - self.manual_min) * 255,
                0, 255
            )
        
        return normalized.astype(np.uint8)
    
    def _apply_colormap(self, frame):
        """Apply color mapping to the frame"""
        if frame is None:
            return None
        
        # Apply colormap
        colored_frame = cv2.applyColorMap(frame, self.colormap)
        
        # Scale up for better visibility
        if self.display_scale > 1:
            height, width = colored_frame.shape[:2]
            new_height = height * self.display_scale
            new_width = width * self.display_scale
            colored_frame = cv2.resize(colored_frame, (new_width, new_height), 
                                     interpolation=cv2.INTER_NEAREST)
        
        return colored_frame
    
    def _add_overlay_info(self, frame, thermal_frame):
        """Add information overlay to the frame"""
        if frame is None or thermal_frame is None:
            return frame
        
        # Calculate FPS
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.fps_start_time
        if elapsed > 1.0:
            fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_start_time = current_time
        else:
            fps = 0
        
        # Add text overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (255, 255, 255)
        thickness = 1
        
        # FPS
        if fps > 0:
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25), font, font_scale, color, thickness)
        
        # Temperature range
        min_temp = np.min(thermal_frame)
        max_temp = np.max(thermal_frame)
        cv2.putText(frame, f"Range: {min_temp} - {max_temp}", (10, 50), font, font_scale, color, thickness)
        
        # Controls info
        cv2.putText(frame, "Controls: 'q'-quit, 'a'-auto scale, 'c'-colormap, 'r'-reset", 
                   (10, frame.shape[0] - 10), font, 0.4, color, thickness)
        
        return frame
    
    def run(self):
        """Run the live display"""
        print("Starting thermal camera display...")
        print("Controls:")
        print("  'q' - Quit")
        print("  'a' - Toggle auto-scaling")
        print("  'c' - Cycle through colormaps")
        print("  'r' - Reset view")
        print("  ESC - Quit")
        
        # Available colormaps
        colormaps = [
            cv2.COLORMAP_JET, cv2.COLORMAP_HOT, cv2.COLORMAP_COOL,
            cv2.COLORMAP_RAINBOW, cv2.COLORMAP_TURBO, cv2.COLORMAP_PLASMA,
            cv2.COLORMAP_VIRIDIS, cv2.COLORMAP_MAGMA
        ]
        colormap_names = ["JET", "HOT", "COOL", "RAINBOW", "TURBO", "PLASMA", "VIRIDIS", "MAGMA"]
        colormap_index = 0
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        try:
            while True:
                # Get latest frame
                thermal_frame = self.camera.get_latest_frame()
                
                if thermal_frame is not None:
                    # Process frame
                    normalized_frame = self._normalize_frame(thermal_frame)
                    colored_frame = self._apply_colormap(normalized_frame)
                    
                    if colored_frame is not None:
                        # Add overlay information
                        display_frame = self._add_overlay_info(colored_frame, thermal_frame)
                        
                        # Display frame
                        cv2.imshow(self.window_name, display_frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == 27:  # 'q' or ESC
                    break
                elif key == ord('a'):
                    self.auto_scale = not self.auto_scale
                    print(f"Auto-scaling: {'ON' if self.auto_scale else 'OFF'}")
                elif key == ord('c'):
                    colormap_index = (colormap_index + 1) % len(colormaps)
                    self.colormap = colormaps[colormap_index]
                    print(f"Colormap: {colormap_names[colormap_index]}")
                elif key == ord('r'):
                    self.auto_scale = True
                    self.colormap = cv2.COLORMAP_JET
                    print("View reset")
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            cv2.destroyAllWindows()

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutting down...")
    sys.exit(0)

def main():
    """Main function"""
    print("FLIR Lepton 3.5 Live Video Display")
    print("==================================")
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Initialize camera
        print("Initializing camera...")
        camera = LeptonCamera()
        
        # Start capture
        camera.start_capture()
        
        # Wait a moment for first frames
        time.sleep(2)
        
        # Initialize display
        display = ThermalDisplay(camera)
        
        # Run display
        display.run()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        if 'camera' in locals():
            camera.close()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
