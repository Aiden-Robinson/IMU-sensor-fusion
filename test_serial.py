"""
Simple IMU Data Reader
Test script to verify serial communication with Arduino
"""

import serial
import time

def test_serial_connection(port='COM3', baudrate=115200):
    """Test serial connection and print IMU data"""
    try:
        # Connect to Arduino
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
        
        print(f"Connected to {port} at {baudrate} baud")
        print("Reading IMU data... (Press Ctrl+C to stop)")
        print("-" * 50)
        
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                print(line)
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

def main():
    print("IMU Serial Test")
    print("Make sure your Arduino is connected and uploading the IMU code")
    
    # Change COM port as needed (check Device Manager on Windows)
    port = input("Enter COM port (e.g., COM3): ").strip()
    if not port:
        port = 'COM3'
    
    test_serial_connection(port)

if __name__ == "__main__":
    main()
