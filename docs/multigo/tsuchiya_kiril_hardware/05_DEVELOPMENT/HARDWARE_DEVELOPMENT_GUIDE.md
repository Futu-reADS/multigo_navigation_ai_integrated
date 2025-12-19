# Hardware Development Guide

**Team:** Tsuchiya (Hardware)
**Date:** 2025-12-16
**Version:** 1.0

## Workflow

1. **Requirements Review** - Understand electrical/mechanical requirements
2. **Design** - Create schematics (KiCAD) or CAD models (SolidWorks)
3. **Review** - Peer review by team lead
4. **Prototype** - Order PCBs, machine parts
5. **Testing** - Verify specifications met
6. **Documentation** - Update schematics, BOM

## File Organization

```
hardware/
├── schematics/
│   ├── safety_controller/
│   ├── power_distribution/
│   └── ...
├── cad/
│   ├── chassis_frame.sldprt
│   ├── swerve_module.sldasm
│   └── ...
├── bom/
│   └── complete_bom.xlsx
└── documentation/
    ├── wiring_harness_spec.xlsx
    └── assembly_instructions.pdf
```

## Version Control

- Schematics: Git LFS for KiCAD files
- CAD: Export STEP files to Git
- Source files: Keep in team shared drive

---
**References:** All Hardware architecture and design documents
