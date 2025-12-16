# Test Execution Guide

**All Teams**
**Date:** 2025-12-16
**Version:** 1.0

## Running Tests

### Vehicle Software (ROS 2)
```bash
# All tests
colcon test

# Specific package
colcon test --packages-select nav_control

# With coverage
colcon test --packages-select nav_control --event-handlers console_cohesion+
colcon test-result --all --verbose
```

### Fleet Management
```bash
# Backend tests
cd fleet_management/tvm-server
npm test

# Frontend tests
cd fleet_management/fleet-ui
npm test

# E2E tests
npm run test:e2e
```

## Test Categories

1. **Unit Tests** - Fast, fully mocked, run on every commit
2. **Integration Tests** - Component interactions, run on PR
3. **System Tests** - End-to-end, run nightly
4. **Field Tests** - Real hardware, run weekly

---
**References:** TESTING_REQUIREMENTS.md (92 requirements)
