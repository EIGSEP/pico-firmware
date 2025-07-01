# TODO - Pico Multi-App Firmware

## Build System
- [x] Create top-level CMakeLists.txt that includes pico_sdk and builds all apps
- [ ] Add CMake subdirectory configuration for each app
- [x] Configure unified build output as `pico_multi.uf2`
- [x] Add build scripts for automated compilation
- [ ] Set up GitHub Actions CI/CD pipeline

## App Integration
- [x] Create header files for each app with proper `*_app()` function declarations
- [x] Modify motor app to expose `motor_app()` instead of `main()`
- [x] Modify switch app to expose `switch_app()` instead of `main()`
- [ ] Add remaining apps via git subtree:
  - [ ] Thermocouple (therm) - Code 0
  - [ ] Sensor monitoring - Code 3
  - [ ] Relay control - Code 4
  - [ ] ADC monitor - Code 5

## USB Configuration
- [x] Implement USB descriptor modification to set serial number based on DIP code
- [ ] Test USB enumeration with PICO_000, PICO_001, etc. format
- [ ] Verify multiple boards enumerate correctly with unique IDs
- [ ] Create udev rules example for Linux hosts

## Hardware
- [ ] Document recommended DIP switch wiring diagram
- [ ] Add pull-up/pull-down resistor specifications
- [ ] Create PCB design for DIP switch breakout (optional)
- [ ] Test with actual hardware across all 6 DIP configurations

## Documentation
- [ ] Update claude.md with actual app implementations
- [ ] Add wiring diagrams for DIP switches
- [ ] Create app-specific documentation for each module
- [ ] Add troubleshooting section for common hardware issues
- [ ] Include example host scripts for each app type

## Testing
- [x] Create test harness for DIP switch reading
- [ ] Implement automated tests for app dispatch logic
- [ ] Add integration tests for each app
- [ ] Test watchdog timer behavior
- [ ] Verify error handling for invalid DIP codes

## Host Software
- [ ] Create Python library for device discovery and communication
- [ ] Implement example scripts for each app type:
  - [ ] Motor control GUI
  - [ ] Switch network controller
  - [ ] Thermocouple data logger
  - [ ] Sensor dashboard
  - [ ] Relay control panel
  - [ ] ADC monitor interface
- [ ] Add device auto-detection and reconnection logic

## Future Enhancements
- [ ] Add LED status indicators for active app
- [ ] Implement app hot-switching (without reboot)
- [ ] Add configuration storage in flash
- [ ] Create bootloader mode for firmware updates
- [ ] Add telemetry/logging framework
- [ ] Implement app-to-app communication protocol

## Known Issues
- [x] ~~USB serial number override not yet implemented (requires TinyUSB customization)~~ ✅ COMPLETED
- [x] ~~Apps currently use their own `main()` functions - need refactoring~~ ✅ COMPLETED  
- [ ] No unified error reporting mechanism across apps

## Notes
- Motor and switch apps are already added via git subtree
- Use documented subtree workflow in SUBTREE_SETUP.md for adding new apps
- Maintain consistent code style across all apps
- Keep individual app functionality isolated and modular