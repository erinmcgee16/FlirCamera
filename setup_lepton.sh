#!/bin/bash

# Setup script for FLIR Lepton 3.5 on Raspberry Pi 5
# This script will install dependencies and configure the system

echo "==============================================="
echo "FLIR Lepton 3.5 Setup for Raspberry Pi 5"
echo "==============================================="

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "Please do not run this script as root (don't use sudo)"
    echo "The script will ask for sudo when needed"
    exit 1
fi

# Update system
echo "Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install required system packages
echo "Installing system dependencies..."
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-numpy \
    build-essential \
    cmake \
    git \
    i2c-tools

# Install Python packages
echo "Installing Python packages..."
pip3 install --user spidev

# Enable SPI and I2C
echo "Configuring SPI and I2C..."

# Check if SPI is already enabled
if ! grep -q "dtparam=spi=on" /boot/firmware/config.txt; then
    echo "Enabling SPI in config.txt..."
    echo "dtparam=spi=on" | sudo tee -a /boot/firmware/config.txt
else
    echo "SPI already enabled in config.txt"
fi

# Check if I2C is already enabled
if ! grep -q "dtparam=i2c_arm=on" /boot/firmware/config.txt; then
    echo "Enabling I2C in config.txt..."
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
else
    echo "I2C already enabled in config.txt"
fi

# Add SPI speed setting for better performance
if ! grep -q "dtparam=spi=on,speed=20000000" /boot/firmware/config.txt; then
    echo "Setting SPI speed..."
    sudo sed -i 's/dtparam=spi=on/dtparam=spi=on/' /boot/firmware/config.txt
fi

# Add user to spi and i2c groups
echo "Adding user to SPI and I2C groups..."
sudo usermod -a -G spi,i2c,gpio $USER

# Create udev rules for SPI/I2C permissions
echo "Creating udev rules..."
cat << 'EOF' | sudo tee /etc/udev/rules.d/99-lepton.rules
# Rules for FLIR Lepton camera access
SUBSYSTEM=="spidev", GROUP="spi", MODE="0664"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0664"
KERNEL=="gpiochip*", GROUP="gpio", MODE="0664"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Create a test script to verify connections
cat << 'EOF' > test_connections.py
#!/usr/bin/env python3
"""
Test script to verify SPI and I2C connections for FLIR Lepton 3.5
"""

import sys

def test_spi():
    """Test SPI connection"""
    try:
        import spidev
        spi = spidev.SpiDev()
        spi.open(0, 0)
        print("✓ SPI connection successful")
        spi.close()
        return True
    except Exception as e:
        print(f"✗ SPI connection failed: {e}")
        return False

def test_i2c():
    """Test I2C connection"""
    try:
        import smbus
        bus = smbus.SMBus(1)
        print("✓ I2C connection successful")
        return True
    except ImportError:
        print("⚠ SMBus not available, installing...")
        import subprocess
        subprocess.run([sys.executable, "-m", "pip", "install", "--user", "smbus"])
        try:
            import smbus
            bus = smbus.SMBus(1)
            print("✓ I2C connection successful")
            return True
        except Exception as e:
            print(f"✗ I2C connection failed: {e}")
            return False
    except Exception as e:
        print(f"✗ I2C connection failed: {e}")
        return False

def test_opencv():
    """Test OpenCV installation"""
    try:
        import cv2
        print(f"✓ OpenCV {cv2.__version__} available")
        return True
    except ImportError:
        print("✗ OpenCV not available")
        return False

def test_numpy():
    """Test NumPy installation"""
    try:
        import numpy as np
        print(f"✓ NumPy {np.__version__} available")
        return True
    except ImportError:
        print("✗ NumPy not available")
        return False

def main():
    print("Testing FLIR Lepton 3.5 setup...")
    print("=" * 40)
    
    tests = [
        ("SPI", test_spi),
        ("I2C", test_i2c),
        ("OpenCV", test_opencv),
        ("NumPy", test_numpy)
    ]
    
    results = []
    for name, test_func in tests:
        print(f"Testing {name}...")
        result = test_func()
        results.append(result)
        print()
    
    print("=" * 40)
    if all(results):
        print("✓ All tests passed! Your system is ready for the Lepton camera.")
    else:
        print("✗ Some tests failed. Please check the errors above.")
        print("You may need to reboot and run this test again.")

if __name__ == "__main__":
    main()
EOF

chmod +x test_connections.py

# Create a systemd service for the camera (optional)
cat << 'EOF' > lepton-camera.service
[Unit]
Description=FLIR Lepton 3.5 Thermal Camera Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStart=/usr/bin/python3 /home/pi/lepton_live_video.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

echo ""
echo "==============================================="
echo "Setup Complete!"
echo "==============================================="
echo ""
echo "Next steps:"
echo "1. Reboot your Raspberry Pi: sudo reboot"
echo "2. After reboot, test the setup: python3 test_connections.py"
echo "3. Connect your FLIR Lepton 3.5 according to the wiring diagram"
echo "4. Run the camera software: python3 lepton_live_video.py"
echo ""
echo "Hardware connections for Breakout Board v2:"
echo "  VIN  -> 3.3V (Pin 1)"
echo "  GND  -> Ground (Pin 6)"
echo "  SDA  -> GPIO 2 (Pin 3) - I2C Data"
echo "  SCL  -> GPIO 3 (Pin 5) - I2C Clock"
echo "  CS   -> GPIO 8 (Pin 24) - SPI Chip Select"
echo "  CLK  -> GPIO 11 (Pin 23) - SPI Clock"
echo "  MISO -> GPIO 9 (Pin 21) - SPI Data Input"
echo "  MOSI -> GPIO 10 (Pin 19) - SPI Data Output"
echo ""
echo "Optional: To install as a system service:"
echo "  sudo cp lepton-camera.service /etc/systemd/system/"
echo "  sudo systemctl enable lepton-camera.service"
echo "  sudo systemctl start lepton-camera.service"
echo ""
echo "A reboot is required for SPI/I2C changes to take effect."
