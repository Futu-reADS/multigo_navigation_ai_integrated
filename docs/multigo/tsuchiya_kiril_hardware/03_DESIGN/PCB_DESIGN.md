# PCB Design Reference

**Team:** Tsuchiya (Hardware)
**Date:** 2025-12-16
**Version:** 1.0

## Custom PCBs

### Safety Controller PCB

**Purpose:** Emergency stop management, watchdog timer

**Components:**
- STM32F4 microcontroller (automotive-grade)
- Dual power supply (24V + 12V for redundancy)
- Emergency stop input circuit (optoisolated)
- Relay driver (60A coil, control 48V main power)
- Watchdog timer (500ms timeout)
- Status LEDs (power, e-stop active, fault)

**Files:**
- Schematic: `pcb/safety_controller/safety_controller_sch.pdf`
- Layout: `pcb/safety_controller/safety_controller_layout.pdf`
- Gerber files: `pcb/safety_controller/gerbers/`

### Power Distribution PCB

**Purpose:** Voltage monitoring, current sensing, fuse holders

**Components:**
- ADC for voltage monitoring (48V, 24V, 12V, 5V rails)
- Hall-effect current sensors (50A range)
- Fuse holders (automotive blade fuses)
- Status LED indicators per rail

**Files:**
- Schematic: `pcb/power_distribution/power_dist_sch.pdf`
- Layout: `pcb/power_distribution/power_dist_layout.pdf`

---
**References:** ELECTRICAL_SYSTEM_ARCHITECTURE.md
