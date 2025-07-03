#!/usr/bin/env python3
"""
Simple test script to verify device query functionality
"""

import serial
import time
import json
import sys

def test_device_query(port):
    """Test the status query command on a specific device"""
    print(f"Testing device on {port}...")
    
    try:
        # Open serial connection
        ser = serial.Serial(port, 115200, timeout=2.0)
        time.sleep(0.2)  # Allow device to initialize
        
        # Clear buffer
        ser.reset_input_buffer()
        
        # Send query command
        print("Sending query command '?'...")
        ser.write(b'?')
        time.sleep(0.1)
        
        # Read response
        response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        print(f"Raw response:\n{response}")
        
        # Parse JSON response
        for line in response.strip().split('\n'):
            if line.strip().startswith('{') and line.strip().endswith('}'):
                try:
                    data = json.loads(line)
                    if data.get('type') == 'status':
                        print("\nParsed status response:")
                        print(f"  App Name: {data.get('app_name')}")
                        print(f"  DIP Code: {data.get('dip_code')}")
                        print(f"  DIP Binary: {data.get('dip_binary')}")
                        print(f"  App Index: {data.get('app_index')}")
                        print(f"  Firmware Version: {data.get('firmware_version')}")
                        return True
                except json.JSONDecodeError as e:
                    print(f"Failed to parse JSON: {e}")
        
        print("No valid status response found")
        ser.close()
        return False
        
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_device_query.py /dev/ttyACM0")
        sys.exit(1)
    
    port = sys.argv[1]
    success = test_device_query(port)
    sys.exit(0 if success else 1)