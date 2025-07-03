#!/usr/bin/env python3
"""
Simple test script to verify Pico status query protocol
"""

import serial
import time
import sys

def test_status_query(port='/dev/ttyACM0'):
    """Test the status query protocol on a Pico device."""
    print(f"Testing status query on {port}...")
    
    try:
        # Open serial connection
        ser = serial.Serial(port, 115200, timeout=2.0)
        time.sleep(0.5)  # Give device time to initialize
        
        # Clear any existing data
        ser.reset_input_buffer()
        
        print("Sending '?' query...")
        # Send query command
        ser.write(b'?')
        ser.flush()
        time.sleep(0.2)  # Give device time to respond
        
        # Read all available data
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"Raw response ({len(response)} bytes):")
            print(repr(response))
            
            # Look for JSON response
            import json
            for line in response.strip().split('\n'):
                line = line.strip()
                if line.startswith('{') and line.endswith('}'):
                    try:
                        data = json.loads(line)
                        print("\nParsed JSON response:")
                        for key, value in data.items():
                            print(f"  {key}: {value}")
                        return True
                    except json.JSONDecodeError as e:
                        print(f"JSON parse error: {e}")
        else:
            print("No response received")
            
        # Try reading normal output
        print("\nReading any other output...")
        ser.write(b'\n')  # Send newline to trigger any output
        time.sleep(0.5)
        if ser.in_waiting > 0:
            other_output = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print("Other output:")
            print(other_output)
            
        ser.close()
        return False
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    print("=" * 60)
    print("Pico Status Query Test")
    print("=" * 60)
    
    if test_status_query(port):
        print("\n✓ Status query protocol is working!")
    else:
        print("\n✗ Status query protocol not detected")
        print("\nPossible reasons:")
        print("1. Old firmware without status query support")
        print("2. App not calling check_for_status_query()")
        print("3. Serial communication issues")
        print("\nSolution: Rebuild and reflash the firmware")