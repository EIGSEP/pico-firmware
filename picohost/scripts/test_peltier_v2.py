#!/usr/bin/env python3
"""
Test script for Peltier temperature control using the picohost library.
"""

import argparse
import json
import time
import sys
from datetime import datetime
from picohost import PicoPeltier


def format_temp_status(data: dict) -> str:
    """Format temperature control status for display."""
    timestamp = datetime.now().strftime("%H:%M:%S")
    
    if data.get('status') == 'update':
        # Temperature control status update
        lines = [f"[{timestamp}] Temperature Control Status:"]
        
        # Channel 1
        temp1 = data.get('temp1', 0)
        target1 = data.get('target1', 0)
        drive1 = data.get('drive1', 0)
        enabled1 = data.get('enabled1', False)
        status1 = "ON" if enabled1 else "OFF"
        
        lines.append(
            f"  Ch1: {temp1:.2f}°C → {target1:.2f}°C "
            f"(drive: {drive1:+.3f}, {status1})"
        )
        
        # Channel 2
        temp2 = data.get('temp2', 0)
        target2 = data.get('target2', 0)
        drive2 = data.get('drive2', 0)
        enabled2 = data.get('enabled2', False)
        status2 = "ON" if enabled2 else "OFF"
        
        lines.append(
            f"  Ch2: {temp2:.2f}°C → {target2:.2f}°C "
            f"(drive: {drive2:+.3f}, {status2})"
        )
        
        return '\n'.join(lines)
    
    elif 'sensors' in data:
        # Temperature monitor status update
        lines = [f"[{timestamp}] Sensor Readings:"]
        
        sensors = data['sensors']
        if isinstance(sensors, str):
            sensors = json.loads(sensors)
            
        for sensor in sensors:
            sensor_id = sensor.get('id', 'Unknown')
            temp = sensor.get('temperature', 0)
            valid = sensor.get('valid', False)
            status = "✓" if valid else "✗"
            
            lines.append(f"  {sensor_id}: {temp:.2f}°C [{status}]")
        
        return '\n'.join(lines)
    
    elif data.get('error'):
        return f"[{timestamp}] ERROR: {data['error']}"
    
    elif data.get('status') == 'initialized':
        sensor_count = data.get('sensor_count', 0)
        return f"[{timestamp}] Device initialized - {sensor_count} sensors found"
    
    else:
        return f"[{timestamp}] {json.dumps(data)}"


def interactive_mode(peltier: PicoPeltier):
    """Run interactive command-line interface."""
    print("\nPeltier Temperature Control")
    print("===========================")
    print("Commands:")
    print("  temp <value> [channel]  - Set target temperature (channel: 0=both, 1/2=individual)")
    print("  hyst <value> [channel]  - Set hysteresis band")
    print("  enable [channel]        - Enable temperature control")
    print("  disable [channel]       - Disable temperature control")
    print("  status                  - Show last status")
    print("  quit                    - Exit")
    print()
    
    while True:
        try:
            cmd_input = input("> ").strip().lower()
            if not cmd_input:
                continue
            
            parts = cmd_input.split()
            cmd = parts[0]
            
            if cmd in ('quit', 'exit', 'q'):
                break
            
            elif cmd == 'temp' and len(parts) >= 2:
                temp = float(parts[1])
                channel = int(parts[2]) if len(parts) > 2 else 0
                if peltier.set_temperature(temp, channel):
                    print(f"Set channel {channel} target to {temp}°C")
            
            elif cmd == 'hyst' and len(parts) >= 2:
                hyst = float(parts[1])
                channel = int(parts[2]) if len(parts) > 2 else 0
                if peltier.set_hysteresis(hyst, channel):
                    print(f"Set channel {channel} hysteresis to ±{hyst}°C")
            
            elif cmd == 'enable':
                channel = int(parts[1]) if len(parts) > 1 else 0
                if peltier.enable(channel):
                    print(f"Enabled channel {channel}")
            
            elif cmd == 'disable':
                channel = int(parts[1]) if len(parts) > 1 else 0
                if peltier.disable(channel):
                    print(f"Disabled channel {channel}")
            
            elif cmd == 'status':
                if peltier.last_status:
                    print("\nLast status:")
                    print(json.dumps(peltier.last_status, indent=2))
                else:
                    print("No status received yet")
            
            else:
                print("Unknown command or invalid syntax")
                
        except KeyboardInterrupt:
            print("\n")
            break
        except ValueError as e:
            print(f"Invalid value: {e}")
        except Exception as e:
            print(f"Error: {e}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Test Peltier temperature control')
    parser.add_argument('-p', '--port', help='Serial port (e.g., /dev/ttyACM0)')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--list', action='store_true', help='List available Pico devices')
    
    # Quick command options
    parser.add_argument('--temp', type=float, help='Set target temperature')
    parser.add_argument('--channel', type=int, default=0, help='Channel (0=both, 1/2=individual)')
    parser.add_argument('--enable', action='store_true', help='Enable control')
    parser.add_argument('--disable', action='store_true', help='Disable control')
    parser.add_argument('--monitor', action='store_true', help='Monitor only (no interactive mode)')
    
    args = parser.parse_args()
    
    # Handle device listing
    if args.list:
        ports = PicoPeltier.find_pico_ports()
        if ports:
            print("Found Pico devices:")
            for port in ports:
                print(f"  {port}")
        else:
            print("No Pico devices found")
        return 0
    
    # Find port if not specified
    if not args.port:
        ports = PicoPeltier.find_pico_ports()
        if not ports:
            print("No Pico devices found. Please check connection.")
            return 1
        elif len(ports) > 1:
            print("Multiple Pico devices found. Please specify port with -p:")
            for port in ports:
                print(f"  {port}")
            return 1
        else:
            args.port = ports[0]
            print(f"Using port: {args.port}")
    
    # Create Peltier controller
    peltier = PicoPeltier(args.port, baudrate=args.baud)
    
    # Set custom response handler for formatted output
    peltier.set_response_handler(lambda data: print(format_temp_status(data)))
    
    # Connect
    if not peltier.connect():
        print(f"Failed to connect to {args.port}")
        return 1
    
    print(f"Connected to Peltier controller on {args.port}")
    peltier.start()
    
    try:
        # Execute quick commands if specified
        if args.temp is not None:
            peltier.set_temperature(args.temp, args.channel)
            time.sleep(0.1)
        
        if args.enable:
            peltier.enable(args.channel)
            time.sleep(0.1)
        
        if args.disable:
            peltier.disable(args.channel)
            time.sleep(0.1)
        
        # Run appropriate mode
        if args.monitor:
            print("Monitoring... Press Ctrl+C to stop")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\nStopping...")
        elif args.temp is None and not args.enable and not args.disable:
            # No quick commands, run interactive mode
            interactive_mode(peltier)
        else:
            # Quick command mode - monitor for a few seconds
            time.sleep(3)
    
    finally:
        peltier.disconnect()
        print("Disconnected")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())