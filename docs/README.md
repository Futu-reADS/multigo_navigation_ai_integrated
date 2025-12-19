# MultiGo Documentation

**Project:** Autonomous Wheelchair Transport Robot (Outdoor-First, Indoor-Compatible)
**Type:** Pilot Project (1-2 vehicles, single facility)
**Technology:** ROS 2 Humble, Swerve Drive, NDT Localization, ArUco Docking
**Last Updated:** December 19, 2025

---

## 🌐 Languages / 言語

**This documentation is available in two languages:**

| Language | Folder | Status | Purpose |
|----------|--------|--------|---------|
| 🇬🇧 **English** | `multigo/` | ✅ Complete (87 files) | **Source of truth** - Use for code work |
| 🇯🇵 **Japanese** | `multigo-ja/` | ✅ Complete (87/87 files - 100%) | Translations for Japanese team members |

**Translation Status:** Fully translated (December 20, 2025)

---

## 📁 Documentation Structure

```
docs/
│
├── 📄 README.md                    ← You are here
├── 📄 CLAUDE.md                    ← AI assistant context guide
│
├── 📁 multigo/                     ← 🇬🇧 ENGLISH (Source of Truth)
│   ├── README.md
│   ├── FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md
│   ├── PRIVACY_LEGAL_REVIEW_PACKAGE_FOR_LI_SAN.md
│   │
│   ├── pankaj_vehicle_software/   [Team 1: ROS 2 Vehicle Software]
│   │   ├── 01_REQUIREMENTS/
│   │   ├── 02_ARCHITECTURE/
│   │   ├── 03_DESIGN/
│   │   ├── 04_INTERFACES/
│   │   ├── 05_DEVELOPMENT/
│   │   └── 06_TESTING/
│   │
│   ├── unno_tvm_server/           [Team 2: TVM Server Backend]
│   │   ├── 01_REQUIREMENTS/
│   │   ├── 02_ARCHITECTURE/
│   │   ├── 03_DESIGN/
│   │   ├── 04_INTERFACES/
│   │   └── 05_DEVELOPMENT/
│   │
│   ├── tsuchiya_kiril_hardware/   [Team 3: Hardware Platform]
│   │   ├── 01_REQUIREMENTS/
│   │   ├── 02_ARCHITECTURE/
│   │   ├── 03_DESIGN/
│   │   ├── 04_INTERFACES/
│   │   ├── 05_DEVELOPMENT/
│   │   └── reference/
│   │
│   └── shared/                    [Interface Contracts - All Teams]
│       └── INTERFACES/
│
└── 📁 multigo-ja/                  ← 🇯🇵 JAPANESE (Translations)
    ├── TRANSLATION_STATUS.md       [翻訳進捗 - Translation progress]
    └── [Same structure as multigo/]
```

---

## 🚀 Quick Start

### For Development Work (Pankaj, All Teams)

**Always use English documentation:**
```bash
cd docs/multigo/

# Read requirements
cat pankaj_vehicle_software/01_REQUIREMENTS/SYSTEM_REQUIREMENTS.md

# Read team guide
cat pankaj_vehicle_software/README.md

# Read interface specs
cat shared/INTERFACES/TVM_API_SPECIFICATION.md
```

**English docs are the source of truth for:**
- Code implementation
- Requirements verification
- Architecture decisions
- Interface contracts
- Testing specifications

### For Japanese Team Members (Unno, Tsuchiya, Kiril)

**Use Japanese documentation for reading:**
```bash
cd docs/multigo-ja/

# 要求仕様を読む / Read requirements
cat pankaj_vehicle_software/01_REQUIREMENTS/SYSTEM_REQUIREMENTS.md

# チームガイドを読む / Read team guide
cat pankaj_vehicle_software/README.md

# 翻訳状況を確認 / Check translation status
cat TRANSLATION_STATUS.md
```

**Important:**
- Japanese docs are translations only
- If English and Japanese differ, English is correct
- Report translation issues to Pankaj

---

## 📚 Key Documents (Read First)

### All Teams
1. **CLAUDE.md** - AI assistant context, bilingual guidelines
2. **multigo/FINAL_SUMMARY_FOR_SENIOR_APPROVAL.md** - Complete project summary
3. **multigo/shared/TEAM_SCOPE_DEFINITION.md** - Who does what (decision tree)

### Team 1: Pankaj (Vehicle Software)
- **multigo/pankaj_vehicle_software/README.md** (507 lines)
- All requirements in `01_REQUIREMENTS/`
- ROS 2 architecture in `02_ARCHITECTURE/`

### Team 2: Unno (TVM Server)
- **multigo/unno_tvm_server/README.md** (410 lines)
- **multigo/unno_tvm_server/TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md** (742 lines - step-by-step guide)
- API specs in `shared/INTERFACES/TVM_API_SPECIFICATION.md`

### Team 3: Tsuchiya + Kiril (Hardware)
- **multigo/tsuchiya_kiril_hardware/README.md** (507 lines)
- **multigo/tsuchiya_kiril_hardware/HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md** (853 lines - complete specs)
- Hardware interfaces in `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md`

---

## 🌐 Bilingual Workflow

### For Claude (AI Assistant)

**CRITICAL RULES:**
1. ✅ **Always read/reference:** `docs/multigo/` (English)
2. ❌ **Never use for code work:** `docs/multigo-ja/` (Japanese)
3. ✅ **Auto-sync translations:** When updating English, also update Japanese
4. ✅ **Keep in sync:** Same file path in both folders

**Example:**
```bash
# Update English (source of truth)
Edit: docs/multigo/pankaj_vehicle_software/01_REQUIREMENTS/SAFETY_REQUIREMENTS.md

# Also update Japanese (translation)
Edit: docs/multigo-ja/pankaj_vehicle_software/01_REQUIREMENTS/SAFETY_REQUIREMENTS.md
```

### For Human Developers

**English developers (Pankaj):**
- Work in `docs/multigo/`
- Update English docs
- Request Japanese translation if needed

**Japanese developers (Unno, Tsuchiya, Kiril):**
- Read from `docs/multigo-ja/` (preferred)
- Can also read `docs/multigo/` (English)
- Report translation issues or missing translations

---

## 📊 Translation Progress

**Current Status:**
- **27/115 files translated (23%)**
- See `multigo-ja/TRANSLATION_STATUS.md` for details

**High Priority Translations (Pending):**
1. unno_tvm_server/TVM_API_IMPLEMENTATION_GUIDE_FOR_UNNO.md
2. tsuchiya_kiril_hardware/HARDWARE_REQUIREMENTS_FOR_TSUCHIYA.md
3. shared/TEAM_SCOPE_DEFINITION.md
4. All team READMEs

**Goal:** 100% translation coverage

---

## ⚠️ Important Notes

### Source of Truth
- **English (`multigo/`) is ALWAYS the source of truth**
- If English and Japanese differ, English is correct
- All code work must reference English docs

### Interface Contracts (Frozen)
These files are **frozen** (no changes without approval):
- `shared/INTERFACES/TVM_API_SPECIFICATION.md`
- `shared/INTERFACES/HARDWARE_SOFTWARE_INTERFACES.md`
- `shared/INTERFACES/TVM_DATA_MODELS.md`

Changes require approval from all affected teams.

### Team Boundaries
- Each team has clear scope (see `shared/TEAM_SCOPE_DEFINITION.md`)
- Cross-team work requires coordination
- Use decision tree to determine ownership

---

## 🔗 Related Resources

**Repository:** `multigo_navigation_ai_integrated`
**ROS 2 Workspace:** `/home/multigo/multigo_ws/`
**Services:** `multigo-vehicle.service`, `multigo-ui.service`

---

## 📧 Questions?

**For documentation questions:**
- Pankaj (Project Lead, Team 1)

**For translation questions:**
- Check `multigo-ja/TRANSLATION_STATUS.md`
- Report missing/incorrect translations to Pankaj

**For technical questions:**
- See team-specific READMEs
- See `shared/TEAM_SCOPE_DEFINITION.md` for responsibility matrix

---

**Last Updated:** December 19, 2025
**Status:** Documentation complete, 23% translated, awaiting senior approval
**Next Phase:** Implementation (18-20 weeks)
