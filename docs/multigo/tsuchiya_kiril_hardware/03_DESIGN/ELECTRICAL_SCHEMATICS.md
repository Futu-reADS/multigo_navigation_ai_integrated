# Electrical Schematics Reference

**Team:** Tsuchiya (Hardware)
**Date:** 2025-12-16
**Version:** 1.0

## Schematic Documents

Detailed electrical schematics are maintained in separate CAD files:

1. **Main Power Distribution** (`schematics/power_distribution.pdf`)
   - 48V battery connections
   - DC-DC converter circuits
   - Fuse and relay placement
   - Emergency stop circuit

2. **Motor Controller Wiring** (`schematics/motor_controllers.pdf`)
   - CAN bus topology
   - Motor phase connections (U, V, W)
   - Hall sensor wiring
   - Power supply connections

3. **Compute Unit Connections** (`schematics/compute_unit.pdf`)
   - Jetson Orin Nano pinout
   - Sensor connections (UART, I2C, SPI)
   - USB hub wiring
   - Ethernet connections

4. **Safety Controller PCB** (`schematics/safety_controller.pdf`)
   - STM32F4 microcontroller circuit
   - E-stop input circuit
   - Relay driver circuit
   - Watchdog timer circuit

5. **Charging Port Interface** (`schematics/charging_port.pdf`)
   - Pogo pin connections
   - Charging relay circuit
   - CAN bus interface
   - Status LED circuit

## Wiring Harness Specifications

See: `documentation/wiring_harness_spec.xlsx`

- Connector types and pin assignments
- Wire gauge and color coding
- Cable lengths and routing
- Crimping specifications

---
**References:** ELECTRICAL_SYSTEM_ARCHITECTURE.md, ELECTRICAL_REQUIREMENTS.md (61 req)
