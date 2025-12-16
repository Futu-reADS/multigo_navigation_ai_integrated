# Deployment Procedures

**All Teams**
**Date:** 2025-12-16
**Version:** 1.0

## Pre-Deployment Checklist

- [ ] All tests pass (unit, integration, system)
- [ ] Security scan complete (no critical vulnerabilities)
- [ ] Documentation updated
- [ ] Backup taken (database, configurations)
- [ ] Rollback plan prepared

## Vehicle Software Deployment

```bash
# 1. Build release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. Install on vehicle
ssh vehicle@192.168.1.100
cd ~/multigo_ws
git pull origin main
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 3. Restart services
sudo systemctl restart multigo-navigation
```

## Fleet Management Deployment

**Backend (TVM Server):**
```bash
# Docker deployment
docker pull registry.example.com/tvm-server:latest
docker-compose up -d
```

**Frontend (Fleet UI):**
```bash
# Build and deploy to S3
npm run build
aws s3 sync dist/ s3://fleet-ui-bucket/
aws cloudfront create-invalidation --distribution-id XYZ --paths "/*"
```

---
**References:** DEPLOYMENT_REQUIREMENTS.md (52 requirements)
