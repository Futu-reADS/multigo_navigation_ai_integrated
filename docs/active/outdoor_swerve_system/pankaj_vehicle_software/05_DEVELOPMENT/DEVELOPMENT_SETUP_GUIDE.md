# Development Setup Guide

**All Teams**  
**Date:** 2025-12-16  
**Version:** 1.0

## Prerequisites

### All Teams
- Git 2.40+
- Docker 24+ and Docker Compose
- VS Code or preferred IDE

### Vehicle Software (Pankaj)
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10+
- C++ compiler (GCC 11+)

### Fleet Management (Unno)
- Node.js 20 LTS
- PostgreSQL 15
- Redis 7.x

### Hardware (Tsuchiya)
- KiCAD 7+ (PCB design)
- SolidWorks or Fusion 360 (CAD)

## Repository Structure

```
multigo_navigation_ai_integrated/
├── src/                  # ROS 2 packages (Vehicle Software)
├── fleet_management/     # TVM Server + Fleet UI
├── hardware/            # Schematics, CAD files
└── docs/                # This documentation
```

## Development Environment Setup

### 1. Clone Repository
```bash
git clone https://github.com/organization/multigo_navigation_ai_integrated.git
cd multigo_navigation_ai_integrated
```

### 2. Team-Specific Setup

**Vehicle Software:**
```bash
cd src/
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
```

**Fleet Management:**
```bash
cd fleet_management/tvm-server/
npm install
cp .env.example .env  # Configure environment variables
docker-compose up -d  # Start PostgreSQL + Redis
npm run dev           # Start development server
```

## Development Workflow

1. Create feature branch: `git checkout -b feature/issue-123-description`
2. Make changes
3. Run tests: `colcon test` (Vehicle) or `npm test` (Fleet)
4. Commit with conventional commits: `feat: add new feature`
5. Push and create pull request
6. Wait for CI/CD checks to pass
7. Request review from team lead

---
**References:** .claude/WORKFLOW.md, CLAUDE.md
