# Mechanical CAD Design Reference

**Team:** Tsuchiya (Hardware)
**Date:** 2025-12-16
**Version:** 1.0

## CAD Files

Mechanical designs are maintained in SolidWorks/Fusion 360:

1. **Main Assembly** (`cad/robot_assembly.sldasm`)
   - Complete robot assembly
   - All subassemblies integrated
   - Bill of Materials (BOM)

2. **Chassis Frame** (`cad/chassis_frame.sldprt`)
   - 40mm × 40mm aluminum extrusion frame
   - Mounting holes and brackets
   - Material: 6061-T6 aluminum

3. **Swerve Drive Module** (`cad/swerve_module.sldasm`)
   - Drive motor assembly
   - Steer motor and belt drive
   - Swivel bearing housing
   - Wheel and tire

4. **Docking Mechanism** (`cad/docking_mechanism.sldasm`)
   - Folding arms (servo-actuated)
   - Engagement pins
   - Alignment guides
   - Sensor mounts

5. **Enclosures** (`cad/enclosures/`)
   - Battery enclosure (IP65)
   - Compute unit enclosure (IP54)
   - Display enclosure
   - All mounting hardware

## Manufacturing Drawings

Technical drawings for fabrication:
- `drawings/chassis_frame.pdf` - Frame fabrication drawing
- `drawings/mounting_brackets.pdf` - Bracket designs
- `drawings/enclosure_panels.pdf` - Panel cutting drawings

---
**References:** MECHANICAL_SYSTEM_ARCHITECTURE.md, MECHANICAL_REQUIREMENTS.md (99 req)
