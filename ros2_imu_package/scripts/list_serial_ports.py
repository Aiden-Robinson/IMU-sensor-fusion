#!/usr/bin/env python3
"""
Simple script to list available serial ports on Linux
"""

import os
import glob
import serial.tools.list_ports


def list_serial_ports():
    """List all available serial ports"""
    print("Available Serial Ports:")
    print("-" * 30)
    
    # Method 1: Using pyserial
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"Port: {port.device}")
        print(f"  Description: {port.description}")
        print(f"  Hardware ID: {port.hwid}")
        print()
    
    # Method 2: Check /dev/ directory for common patterns
    print("Checking /dev/ directory:")
    patterns = ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyS*']
    
    for pattern in patterns:
        devices = glob.glob(pattern)
        if devices:
            print(f"{pattern}: {devices}")
    
    if not ports and not any(glob.glob(pattern) for pattern in patterns):
        print("No serial ports found!")
        print("Make sure your Arduino is connected and recognized by the system.")
        print("You can also try: ls /dev/tty*")


if __name__ == "__main__":
    list_serial_ports()
