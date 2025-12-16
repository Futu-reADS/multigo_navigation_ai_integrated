# Test Automation Setup

**All Teams**
**Date:** 2025-12-16
**Version:** 1.0

## CI/CD Pipeline (GitHub Actions)

```yaml
# .github/workflows/test.yml
name: Test
on: [push, pull_request]
jobs:
  vehicle-tests:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Install ROS 2 Humble
        run: ...
      - name: Build
        run: colcon build
      - name: Test
        run: colcon test
      
  fleet-tests:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: 20
      - name: Install dependencies
        run: npm ci
      - name: Run tests
        run: npm test
```

## Test Reports

- Unit test results published to GitHub Actions
- Coverage reports uploaded to Codecov
- Failed tests block merge

---
**References:** TESTING_REQUIREMENTS.md
