# Implementation Timeline 2026

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Period:** January 15 - July 5, 2026 (24 weeks)
**Date:** December 16, 2025
**Version:** 1.0

---

## Timeline Overview (Gantt Chart)

```
Team       Jan   Feb   Mar   Apr   May   Jun   Jul
           15    01    01    01    01    01    05
────────────────────────────────────────────────────────
Hardware   [========Phase 0========>]
(Okinawa)  [Sensors][Arch][PC][Swerve][Final][Dock][Ctrl]
           └──────────────────────────────┘
                                          [Support/Maintenance──────────>]

Vehicle SW [======Development======>][==Test==][Integration>]
(Pankaj)   [Sw][Nav][Per][Dk][Safe][UI][eHMI][Unit Tests][Integration]
           └───────────────────────────────────────────────┘
                                                           [Field Test──>]

Fleet Mgmt [==============TVM Development=================>]
(Unno)     [Route][Resv][Admin][Map][Login][Walk][Call]
           └─────────────────────────────────────────────────┘

Field Test                                [==All Systems==>]
(All)                                     [17 May - 05 Jul]
```

---

## Phase Breakdown

### **Phase 0: Hardware Modification** (Jan 15 - May 1, 2026)
**Owner:** Okinawa (Tsuchiya) + Kirill (CAD)
**Duration:** 15.5 weeks

| Task | Duration | Dates | Deliverable |
|------|----------|-------|-------------|
| Sensor selection & layout | 2 weeks | Jan 15-28 | Sensor list, mounting plan |
| Hardware architecture | 2 weeks | Jan 29 - Feb 11 | System architecture doc |
| PC selection (specs) | 2 days | Feb 12-13 | Jetson Orin Nano confirmed |
| Swerve drive mechanism (w/ suspension) | 3 weeks | Feb 14 - Mar 6 | Prototype module |
| Final hardware (LiDAR mounted) | 2 weeks | Mar 7-20 | Assembled platform |
| Docking mechanism | 2 weeks | Mar 21 - Apr 3 | Docking prototype |
| Control system | 4 weeks | Apr 4 - May 1 | Motor controllers integrated |
| Exterior CAD (Kirill) | 3.5 weeks | Mar 2-27 | CAD files for outsourcing |
| Hardware support/maintenance | 14 weeks | Mar 27 - Jul 2 | Ongoing support |

**Critical Milestone:** Mar 20 - Hardware platform ready for software integration

---

### **Phase 1: Vehicle Software Development** (Jan 15 - May 17, 2026)
**Owner:** Pankaj
**Duration:** 18 weeks

#### Development (Jan 15 - Mar 29)
| Module | Duration | Dates | Integration Ready |
|--------|----------|-------|-------------------|
| Swerve Drive | 11 days | Jan 15-25 | ✓ |
| Navigation | 11 days | Jan 25 - Feb 5 | ✓ |
| Perception | 10 days | Feb 5-15 | ✓ |
| Docking | 11 days | Feb 15-26 | ✓ |
| Safety | 10 days | Feb 26 - Mar 8 | ✓ |
| UI | 11 days | Mar 8-19 | ✓ |
| eHMI | 10 days | Mar 19-29 | ✓ |

#### Unit Testing (Mar 29 - Apr 23)
| Module | Test Duration | Dates |
|--------|---------------|-------|
| Swerve Drive | 4 days | Mar 29 - Apr 2 |
| Navigation | 4 days | Apr 2-5 |
| Perception | 4 days | Apr 5-9 |
| Docking | 4 days | Apr 9-12 |
| Safety | 4 days | Apr 12-16 |
| UI | 4 days | Apr 16-19 |
| eHMI | 4 days | Apr 19-23 |

#### Integration Testing (Apr 23 - May 17)
| Module | Integration Duration | Dates |
|--------|---------------------|-------|
| Swerve Drive | 3 days | Apr 23-26 |
| Navigation | 4 days | Apr 26-30 |
| Perception | 3 days | Apr 30 - May 3 |
| Docking | 4 days | May 3-7 |
| Safety | 3 days | May 7-10 |
| UI | 4 days | May 10-14 |
| eHMI | 3 days | May 14-17 |

**Critical Milestone:** May 17 - Vehicle software ready for field testing

---

### **Phase 2: Fleet Management (TVM) Development** (Jan 16 - Jun 4, 2026)
**Owner:** Unno
**Duration:** 20 weeks

| Feature | Duration | Dates | Priority |
|---------|----------|-------|----------|
| Route setting | 12 days | Jan 16-27 | Critical |
| Reservation system | 3 days | Jan 28-30 | Critical |
| Reception function | 5 days | Jan 31 - Feb 4 | High |
| Cancellation function | 5 days | Feb 5-9 | High |
| Emergency setting | 3 days | Feb 10-12 | Critical |
| Admin registration | 7 days | Feb 13-19 | High |
| Map confirmation | 7 days | Feb 20-26 | High |
| Battery display | 5 days | Feb 27 - Mar 3 | High |
| Floor map registration | 10 days | Mar 4-13 | Medium |
| Room number setting | 12 days | Mar 14-25 | Medium |
| Resident info setting | 12 days | Mar 26 - Apr 6 | Medium |
| Login function | 3 days | Apr 7-9 | Critical |
| Permission/RBAC | 5 days | Apr 10-14 | Critical |
| Return home mode | 8 days | Apr 15-22 | High |
| Walking reservation | 14 days | Apr 23 - May 6 | Critical |
| Call function | 7 days | May 7-13 | High |
| Advance notification | 5 days | May 14-18 | Medium |
| Status confirmation | 3 days | May 19-21 | High |
| Voice call function | 14 days | May 22 - Jun 4 | Medium |

**Critical Milestone:** Jun 4 - TVM system feature-complete

---

### **Phase 3: Field Testing** (May 17 - Jul 5, 2026)
**Owner:** Pankaj + Kirill (On-site)
**Duration:** 7 weeks

| Test Category | Duration | Dates | Location |
|---------------|----------|-------|----------|
| Swerve Drive | 1 week | May 17-24 | Test site |
| Navigation | 1 week | May 24-31 | Outdoor paths |
| Perception | 1 week | May 31 - Jun 7 | Various conditions |
| Docking | 1 week | May 24-31 | Wheelchair docking |
| Safety | 1 week | May 31 - Jun 7 | Safety scenarios |
| UI/eHMI | 1 week | Jun 7-14 | User interaction |
| **System Integration** | 1.5 weeks | Jun 14-25 | Full system |
| **Outdoor Field Validation (Final)** | 1.5 weeks | Jun 25 - Jul 5 | Real-world deployment |

**Critical Milestone:** Jul 5 - System validated for pilot deployment

---

## Parallel Development Tracks

```
Timeline:  Jan 15 ────────────> May 17 ──────> Jul 5
                                    ↓            ↓
Hardware:  [===== Phase 0 =====] Ready    [Support]
                                    ↓
Vehicle:   [=== Dev ==][Test][Int] Ready ──>[Field]
                                    ↓
Fleet:     [======= TVM Dev =======] Ready ─>[Test]
                                              ↓
                                         [Integration]
```

**Key Dependencies:**
1. **Mar 20:** Hardware ready → Vehicle software can test on real hardware
2. **May 17:** Vehicle software ready → Field testing begins
3. **Jun 4:** TVM ready → Full system integration testing
4. **Jun 14:** All systems ready → Final outdoor validation

---

## Critical Path

```
Hardware (Phase 0) → Vehicle SW Integration → Field Testing → System Validation
   15 weeks              18 weeks              7 weeks         Final
```

**Total Duration:** 24 weeks (Jan 15 - Jul 5, 2026)

---

## Resource Allocation

| Team Member | Primary Responsibility | Weeks | Critical Period |
|-------------|------------------------|-------|-----------------|
| **Okinawa (Tsuchiya)** | Hardware platform | 15.5 | Jan 15 - May 1 |
| **Kirill** | CAD design, Field support | 3.5 + 14 | Mar 2-27, May 17 - Jul 5 |
| **Pankaj** | Vehicle software, Field test | 18 + 7 | Jan 15 - Jul 5 |
| **Unno** | Fleet management (TVM) | 20 | Jan 16 - Jun 4 |

---

## Risk Mitigation

| Risk | Mitigation | Buffer |
|------|------------|--------|
| Hardware delays | Start vehicle SW with simulation | 2 weeks buffer built in |
| Integration issues | Weekly sync meetings between teams | Continuous integration |
| Field test weather | Indoor backup test facility | Flexible scheduling |

---

## Key Milestones Summary

| Date | Milestone | Owner |
|------|-----------|-------|
| **Jan 28** | Sensors selected | Okinawa |
| **Feb 13** | PC selected (Jetson Orin Nano) | Okinawa |
| **Mar 6** | Swerve drive prototype ready | Okinawa |
| **Mar 20** | Hardware platform complete | Okinawa |
| **Mar 27** | Exterior CAD complete | Kirill |
| **Mar 29** | Vehicle SW development complete | Pankaj |
| **Apr 23** | Vehicle SW unit tests complete | Pankaj |
| **May 1** | Hardware control system complete | Okinawa |
| **May 17** | Vehicle SW integration complete | Pankaj |
| **Jun 4** | TVM feature-complete | Unno |
| **Jun 25** | System integration complete | All |
| **Jul 5** | Final outdoor validation complete | All |

---

## Success Criteria

- [ ] Hardware platform operational by Mar 20
- [ ] Vehicle software integrated by May 17
- [ ] TVM system operational by Jun 4
- [ ] All field tests passed by Jun 25
- [ ] Final validation complete by Jul 5
- [ ] Zero critical safety issues
- [ ] System ready for pilot deployment

---

**Document End**
