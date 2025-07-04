# Tempctrl Simplification Plan

## Current Status
- **tempmon simplification**: ✅ COMPLETED and committed to git
- **tempctrl simplification**: 🔄 IN PROGRESS

## Analysis Results

### Commands Currently Handled
1. **`set_temp`** - Sets target temperature for channels 1, 2, or both ✅ KEEP
2. **`set_hysteresis`** - Sets hysteresis band for channels 1, 2, or both ✅ KEEP 
3. **`enable`** - Enables temperature control for channels 1, 2, or both ✅ KEEP
4. **`disable`** - Disables temperature control and stops PWM output ✅ KEEP

### Threading Analysis
- **Multicore header included** but no multicore functions used ❌ REMOVE
- No actual threading implementation found

### Dead Code Identified
- `#include "pico/multicore.h"` - unused header
- `t_target` field - set but never used in control logic
- `T_prev` field - set but never used in control logic  
- `t_prev` field - set but never used in control logic
- `t_now` field - set but never used in control logic
- `gain` field - set but never used in current control logic
- `tempctrl_update_temperature()` function - only updates unused time fields

### Essential Code to Keep
- **Hysteresis control** - actively used in `tempctrl_hysteresis_drive()` 
- **Channel selection logic** - may be simplified but currently functional
- **All current commands** - all are essential for operation

## Todo List

### High Priority
- [ ] Verify tempctrl still compiles and works after simplifications
- [ ] Test that hysteresis control still works properly

### Medium Priority  
- [ ] Remove unused time fields from TempControl struct (t_target, T_prev, t_prev, t_now)
- [ ] Remove unused gain field from TempControl struct
- [ ] Analyze if channel selection logic can be simplified
- [ ] Clean up tempctrl_update_temperature function if time fields are removed

### Low Priority
- [ ] Remove unused multicore.h header from tempctrl.c

## Files to Modify
- `src/tempctrl.c` - main implementation
- `src/tempctrl.h` - struct definition and headers

## Key Insight
**Hysteresis is essential** - prevents rapid on/off cycling when temperature oscillates around setpoint. Without it, Peltier elements would constantly switch causing hardware damage and energy waste.

## Next Steps
1. Start with low-risk changes (remove unused header)
2. Remove unused struct fields
3. Clean up functions that only use removed fields
4. Test compilation and basic functionality
5. Verify hysteresis control still works as expected