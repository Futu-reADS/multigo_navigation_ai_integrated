# Site Commissioning Guide

**All Teams**
**Date:** 2025-12-16
**Version:** 1.0

## Commissioning Steps

### 1. Infrastructure Setup
- [ ] Install charging station (200V 3-phase power)
- [ ] Configure WiFi access points (coverage map)
- [ ] Set up LTE connectivity (SIM cards installed)

### 2. Vehicle Setup
- [ ] Unpack and assemble vehicles
- [ ] Charge batteries to 100%
- [ ] Configure vehicle IDs (VEH-001, VEH-002, ...)
- [ ] Calibrate sensors (LiDAR, cameras, IMU)

### 3. Software Configuration
- [ ] Deploy TVM server
- [ ] Configure database (create tables, seed data)
- [ ] Configure geofences (operational boundaries)
- [ ] Test fleet UI access (all user roles)

### 4. Mapping
- [ ] Create map (manual SLAM or pre-mapping)
- [ ] Mark waypoints (charging station, common destinations)
- [ ] Verify NDT localization accuracy (±10cm)

### 5. System Integration Test
- [ ] Vehicle → TVM communication (telemetry, commands)
- [ ] Manual test mission (navigate A→B)
- [ ] Automated test mission (reservation → mission → completion)
- [ ] Emergency stop test (physical + UI + remote)

### 6. Site Acceptance Test (SAT)
- [ ] Customer witnesses test missions
- [ ] Performance verification (speed, accuracy, battery life)
- [ ] Safety demonstration (obstacle avoidance, e-stop)
- [ ] Customer sign-off

---
**References:** DEPLOYMENT_REQUIREMENTS.md
