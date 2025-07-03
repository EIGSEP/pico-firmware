"""
Pico Device Scanner
Scans for all connected Raspberry Pi Pico devices and identifies which
app they're running.
"""

import serial
import serial.tools.list_ports
import time
import sys
import subprocess
import re
from typing import List, Dict, Optional, Set


def get_lsusb_pico_devices() -> Set[str]:
    """Get Pico device identifiers from lsusb output."""
    pico_identifiers = set()
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        if result.returncode == 0:
            # Look for Raspberry Pi devices (vendor ID 2e8a)
            for line in result.stdout.splitlines():
                if '2e8a:' in line.lower():
                    # Extract bus and device number
                    match = re.match(r'Bus (\d+) Device (\d+):', line)
                    if match:
                        bus, device = match.groups()
                        pico_identifiers.add(f"{bus}:{device}")
                        
    except (subprocess.SubprocessError, FileNotFoundError):
        pass  # lsusb might not be available on all systems
    
    return pico_identifiers


def find_pico_devices() -> List[serial.tools.list_ports_common.ListPortInfo]:
    """Find all Raspberry Pi Pico devices connected via USB."""
    pico_devices = []
    
    # Common VID/PID combinations for Raspberry Pi Pico
    pico_vid_pid = [
        (0x2E8A, 0x0005),  # Raspberry Pi Pico
        (0x2E8A, 0x000A),  # Raspberry Pi Pico W
        (0x2E8A, 0x0003),  # Raspberry Pi Pico (bootloader)
        (0x2E8A, 0x0004),  # Raspberry Pi Pico (MicroPython)
        (0x2E8A, 0x000F),  # Raspberry Pi Pico 2
    ]
    
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        is_pico = False
        
        # Check by VID/PID (most reliable)
        if port.vid and port.pid:
            if (port.vid, port.pid) in pico_vid_pid:
                is_pico = True
        
        # For ttyACM devices without VID/PID, check more carefully
        if not is_pico and '/dev/ttyACM' in port.device:
            # Check description
            if port.description and 'pico' in port.description.lower():
                is_pico = True
            # Check manufacturer
            elif port.manufacturer and 'raspberry' in port.manufacturer.lower():
                is_pico = True
            # Check product
            elif port.product and 'pico' in port.product.lower():
                is_pico = True
        
        if is_pico:
            pico_devices.append(port)
            
    return pico_devices


def read_device_info(port: str, timeout: float = 2.0) -> Dict[str, str]:
    """
    Read device information from a Pico device.
    Sends a query command and reads the response.
    """
    info = {
        'port': port,
        'app': 'Unknown',
        'id': 'Unknown',
        'status': 'Unknown'
    }
    
    try:
        # Open serial connection
        ser = serial.Serial(port, 115200, timeout=timeout)
        time.sleep(0.1)  # Give device time to initialize
        
        # Clear any existing data
        ser.reset_input_buffer()
        
        # Send query command (you may need to adjust this based on your firmware)
        # For now, we'll just read any initial output
        ser.write(b'\n')  # Send newline to trigger any welcome message
        time.sleep(0.1)
        
        # Read response
        response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        
        # Parse response for app name and device ID
        lines = response.strip().split('\n')
        for line in lines:
            if 'app' in line.lower():
                info['app'] = line.strip()
            elif 'id' in line.lower() or 'device' in line.lower():
                info['id'] = line.strip()
        
        # If no specific info found, store raw response
        if info['app'] == 'Unknown' and response:
            info['app'] = response[:50].strip()  # First 50 chars
            
        info['status'] = 'Connected'
        ser.close()
        
    except serial.SerialException as e:
        info['status'] = f'Error: {str(e)}'
    except Exception as e:
        info['status'] = f'Error: {str(e)}'
        
    return info


def scan_all_devices(show_lsusb: bool = False):
    """Scan all connected Pico devices and display their information."""
    print("Scanning for Raspberry Pi Pico devices...")
    print("-" * 70)
    
    # Optionally show lsusb info
    if show_lsusb:
        lsusb_picos = get_lsusb_pico_devices()
        if lsusb_picos:
            print(f"lsusb found {len(lsusb_picos)} Raspberry Pi device(s)")
            print()
    
    devices = find_pico_devices()
    
    if not devices:
        print("No Pico devices found.")
        return
    
    print(f"Found {len(devices)} Pico device(s):\n")
    
    for i, device in enumerate(devices, 1):
        print(f"Device {i}:")
        print(f"  Port: {device.device}")
        print(f"  Description: {device.description}")
        print(f"  VID:PID: {device.vid:04X}:{device.pid:04X}" if device.vid else "  VID:PID: Unknown")
        print(f"  Serial Number: {device.serial_number if device.serial_number else 'Unknown'}")
        print(f"  Manufacturer: {device.manufacturer if device.manufacturer else 'Unknown'}")
        print(f"  Product: {device.product if device.product else 'Unknown'}")
        
        # Try to read device-specific info
        print("  Reading device info...")
        info = read_device_info(device.device)
        
        print(f"  Status: {info['status']}")
        if info['status'] == 'Connected':
            print(f"  Running App: {info['app']}")
            print(f"  Device ID: {info['id']}")
        
        print("-" * 70)


def monitor_devices(interval: float = 5.0):
    """Continuously monitor Pico devices."""
    print(f"Monitoring Pico devices (checking every {interval} seconds)...")
    print("Press Ctrl+C to stop.\n")
    
    known_devices = set()
    
    try:
        while True:
            current_devices = find_pico_devices()
            current_ports = {d.device for d in current_devices}
            
            # Check for new devices
            for device in current_devices:
                if device.device not in known_devices:
                    print(f"\n[NEW] Device connected on {device.device}")
                    info = read_device_info(device.device)
                    print(f"      App: {info['app']}")
                    known_devices.add(device.device)
            
            # Check for removed devices
            removed = known_devices - current_ports
            for port in removed:
                print(f"\n[REMOVED] Device disconnected from {port}")
                known_devices.remove(port)
            
            time.sleep(interval)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Scan and identify Raspberry Pi Pico devices')
    parser.add_argument('-m', '--monitor', action='store_true', 
                        help='Continuously monitor for device changes')
    parser.add_argument('-i', '--interval', type=float, default=5.0,
                        help='Monitor interval in seconds (default: 5.0)')
    parser.add_argument('-l', '--lsusb', action='store_true',
                        help='Also show lsusb output for comparison')
    
    args = parser.parse_args()
    
    if args.monitor:
        monitor_devices(args.interval)
    else:
        scan_all_devices(show_lsusb=args.lsusb)


if __name__ == '__main__':
    main()
