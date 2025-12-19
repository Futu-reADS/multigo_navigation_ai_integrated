# Deployment Architecture (Vehicle Software)

**Project:** Outdoor Wheelchair Transport Robot
**Document Type:** Architecture Specification
**Status:** Active
**Version:** 2.0 (Simplified for Pilot)
**Last Updated:** 2025-12-19
**Owner:** Pankaj (Vehicle Software)
**Scope:** Vehicle software deployment ONLY

---

## Table of Contents

1. [Overview](#1-overview)
2. [Vehicle Compute Platform](#2-vehicle-compute-platform)
3. [Software Stack](#3-software-stack)
4. [Deployment Process](#4-deployment-process)
5. [Configuration Management](#5-configuration-management)
6. [Update Mechanism](#6-update-mechanism)
7. [Monitoring & Logging](#7-monitoring--logging)

---

## 1. Overview

### 1.1 Purpose

This document defines how vehicle software is deployed to the onboard compute unit.

**In Scope:**
- Vehicle software installation (ROS 2, dependencies, custom packages)
- Systemd service configuration
- Configuration file deployment
- Update mechanism (manual for pilot, OTA in future)
- Local logging setup

**Out of Scope:**
- TVM server deployment (Unno's responsibility)
- Database deployment (Unno's responsibility)
- Cloud infrastructure (Unno's responsibility)
- Hardware installation (Tsuchiya's responsibility)
- Multi-region deployment (not needed for pilot)

### 1.2 Deployment Topology

```
┌─────────────────────────────────────────────────────────┐
│                    TVM Server                           │
│                 (Unno's Deployment)                      │
│         See: unno_tvm_server/DEPLOYMENT_*.md            │
└──────────────────┬──────────────────────────────────────┘
                   │
                   │ HTTPS/WSS (TVM API)
                   │
┌──────────────────▼──────────────────────────────────────┐
│                Vehicle Compute Unit                      │
│          (This Document's Scope)                         │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │  Operating System: Ubuntu 22.04 LTS            │    │
│  └────────────────────────────────────────────────┘    │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │  Middleware: ROS 2 Humble                      │    │
│  └────────────────────────────────────────────────┘    │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │  Vehicle Software (ROS 2 Workspace)            │    │
│  │  - Navigation (Nav2, NDT localization)         │    │
│  │  - Docking (ArUco visual servoing)             │    │
│  │  - Perception (LiDAR processing)               │    │
│  │  - Control (Swerve drive controller)           │    │
│  │  - Safety (Emergency stop monitor)             │    │
│  │  - TVM Client (REST + WebSocket)               │    │
│  │  - Local UI (React app)                        │    │
│  └────────────────────────────────────────────────┘    │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │  Systemd Services                               │    │
│  │  - multigo-vehicle.service (main)               │    │
│  │  - multigo-ui.service (local UI)                │    │
│  └────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────┘
```

---

## 2. Vehicle Compute Platform

### 2.1 Hardware Specification

**Compute Unit:** GMKtec Nucbox K6
- **CPU:** AMD Ryzen 7 7840HS (8 cores, 16 threads, 3.8-5.1 GHz)
- **RAM:** 32GB DDR5
- **GPU:** AMD Radeon 780M (integrated)
- **Storage:** 1TB NVMe SSD
- **Ports:** USB 3.2, USB-C, Ethernet, HDMI

**Provided By:** Tsuchiya (Hardware team)
**Software Team Responsibility:** Install OS and software only

### 2.2 Operating System

**OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Kernel:** 5.15+ (default Ubuntu 22.04 kernel)
- **Architecture:** x86_64
- **Installation Method:** Standard Ubuntu Server or Desktop installation

**Why Ubuntu 22.04:**
- ROS 2 Humble officially supports Ubuntu 22.04
- Long-term support until April 2027
- Extensive hardware driver support
- Large community

---

## 3. Software Stack

### 3.1 System Dependencies

**Install these on fresh Ubuntu 22.04:**

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble dependencies
sudo apt install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv

# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop  # Or ros-humble-base for headless

# Install ROS 2 development tools
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 3.2 Vehicle Software Dependencies

**ROS 2 packages:**
- `ros-humble-navigation2` - Nav2 navigation stack
- `ros-humble-nav2-bringup` - Nav2 launch files
- `ros-humble-robot-localization` - EKF, UKF localization
- `ros-humble-pcl-ros` - Point cloud processing
- `ros-humble-cv-bridge` - OpenCV-ROS bridge
- `ros-humble-image-transport` - Camera image transport

**Python dependencies:**
```bash
pip3 install \
    opencv-python \
    numpy \
    scipy \
    requests \
    websockets \
    pyyaml \
    pyjwt
```

**See:** `DEVELOPMENT_SETUP_GUIDE.md` for complete dependency list

### 3.3 Vehicle Software Workspace

**Workspace structure:**
```
/home/multigo/multigo_ws/
├── src/
│   ├── nav_control/       # Navigation controller
│   ├── nav_docking/       # Docking controller
│   ├── nav_goal/          # Goal manager
│   ├── perception/        # LiDAR processing
│   ├── swerve_drive/      # Swerve drive controller
│   ├── safety_monitor/    # Safety monitor
│   ├── tvm_client/        # TVM API client
│   └── local_ui/          # React local UI
├── build/                 # Build artifacts (ignored in git)
├── install/               # Installed packages
└── log/                   # Build logs
```

**Build command:**
```bash
cd /home/multigo/multigo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 4. Deployment Process

### 4.1 Initial Deployment (Fresh Install)

**Step 1: Install Ubuntu 22.04**
- Create bootable USB with Ubuntu 22.04 Server
- Boot vehicle compute unit from USB
- Install Ubuntu (guided installation)
- Create user: `multigo` (password set during installation)
- Enable SSH server during installation

**Step 2: Install System Dependencies**
```bash
ssh multigo@<vehicle-ip>
sudo apt update && sudo apt upgrade -y
# Run system dependency installation from Section 3.1
```

**Step 3: Clone Vehicle Software Repository**
```bash
mkdir -p /home/multigo/multigo_ws/src
cd /home/multigo/multigo_ws/src
git clone https://github.com/<org>/multigo_navigation.git
```

**Step 4: Install ROS Dependencies**
```bash
cd /home/multigo/multigo_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Step 5: Build Vehicle Software**
```bash
cd /home/multigo/multigo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

**Step 6: Deploy Configuration Files**
```bash
sudo mkdir -p /etc/multigo
sudo cp /home/multigo/multigo_ws/src/multigo_navigation/config/vehicle_config.yaml \
    /etc/multigo/config.yaml
sudo chmod 600 /etc/multigo/config.yaml
sudo chown multigo:multigo /etc/multigo/config.yaml

# Edit config with vehicle-specific values
sudo nano /etc/multigo/config.yaml
# Set: vehicle_id, tvm_server_url, api_key
```

**Step 7: Install Systemd Services**
```bash
sudo cp /home/multigo/multigo_ws/src/multigo_navigation/deploy/multigo-vehicle.service \
    /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable multigo-vehicle
sudo systemctl start multigo-vehicle
```

**Step 8: Verify Deployment**
```bash
sudo systemctl status multigo-vehicle
journalctl -u multigo-vehicle -f  # Check logs
rostopic list  # Verify ROS topics are running
```

### 4.2 Systemd Service Configuration

**File:** `/etc/systemd/system/multigo-vehicle.service`

```ini
[Unit]
Description=Multigo Vehicle Software
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=multigo
Group=multigo
WorkingDirectory=/home/multigo/multigo_ws
Environment="ROS_DOMAIN_ID=0"
Environment="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /home/multigo/multigo_ws/install/setup.bash && \
    ros2 launch multigo_bringup vehicle.launch.py"
Restart=always
RestartSec=10s

[Install]
WantedBy=multi-user.target
```

**Key points:**
- Runs as `multigo` user (not root)
- Auto-restart on failure (10s delay)
- Sources ROS 2 and workspace setups
- Starts main vehicle launch file

---

## 5. Configuration Management

### 5.1 Configuration File Location

**Main config:** `/etc/multigo/config.yaml`

**Example structure:**
```yaml
vehicle:
  id: "VH-001"  # Unique vehicle ID
  model: "MultiGo-Outdoor-v1"

tvm:
  server_url: "https://tvm.example.com"
  api_key_file: "/etc/multigo/api_key.secret"
  websocket_url: "wss://tvm.example.com/ws"
  heartbeat_interval: 1.0  # seconds

navigation:
  max_velocity: 1.5  # m/s
  max_acceleration: 0.5  # m/s²

docking:
  aruco_marker_id: 42
  target_distance: 0.05  # meters (±5mm precision)

safety:
  estop_timeout: 0.1  # seconds (100ms)
  obstacle_detection_distance: 1.0  # meters

logging:
  level: "INFO"  # DEBUG, INFO, WARNING, ERROR
  file: "/var/log/multigo/vehicle.log"
  max_size_mb: 100
  rotation_count: 5
```

### 5.2 Secrets Management

**API keys stored separately:**
```bash
# Create secret file
sudo touch /etc/multigo/api_key.secret
sudo chmod 600 /etc/multigo/api_key.secret
sudo chown multigo:multigo /etc/multigo/api_key.secret
echo "<API_KEY_FROM_TVM>" | sudo tee /etc/multigo/api_key.secret
```

**Referenced in config:**
```yaml
tvm:
  api_key_file: "/etc/multigo/api_key.secret"
```

**Why separate files:**
- Config file can be committed to git (no secrets)
- Secret file has stricter permissions (600)
- Easier to rotate secrets

---

## 6. Update Mechanism

### 6.1 Manual Update (Pilot)

**For pilot, updates are manual via SSH:**

```bash
# Step 1: SSH into vehicle
ssh multigo@<vehicle-ip>

# Step 2: Stop service
sudo systemctl stop multigo-vehicle

# Step 3: Update code
cd /home/multigo/multigo_ws/src/multigo_navigation
git pull origin main

# Step 4: Rebuild
cd /home/multigo/multigo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Step 5: Restart service
sudo systemctl start multigo-vehicle

# Step 6: Verify
sudo systemctl status multigo-vehicle
journalctl -u multigo-vehicle -n 50  # Check recent logs
```

### 6.2 Future: OTA Updates (Post-Pilot)

**Planned for production (not pilot):**
- TVM server pushes update packages
- Vehicle downloads update in background
- Update applied during maintenance window
- Rollback capability if update fails

**Not implementing for pilot because:**
- Adds complexity
- Risk of bricking vehicle remotely
- Manual updates acceptable for 1-2 vehicles

---

## 7. Monitoring & Logging

### 7.1 Local Logging

**System logs (systemd journal):**
```bash
# View vehicle service logs
journalctl -u multigo-vehicle -f

# View recent errors
journalctl -u multigo-vehicle -p err -n 100

# View logs since boot
journalctl -u multigo-vehicle --boot
```

**Application logs:**
- **Location:** `/var/log/multigo/`
- **Files:**
  - `vehicle.log` - Main application log
  - `navigation.log` - Navigation subsystem
  - `safety.log` - Safety monitor (E-stop events, collisions)
  - `tvm_client.log` - TVM API communication

**Log rotation:**
```bash
# /etc/logrotate.d/multigo
/var/log/multigo/*.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 0644 multigo multigo
}
```

### 7.2 Remote Monitoring (TVM Integration)

**Vehicle sends telemetry to TVM:**
- **Location:** Sent via REST API (`POST /api/v1/telemetry`)
- **Frequency:** 1 Hz
- **Data:**
  - Vehicle status (IDLE, NAVIGATING, DOCKED, ERROR)
  - Battery level
  - Current location (x, y, heading)
  - Active errors
  - CPU/RAM usage

**Logs sent to TVM:**
- **Critical errors only** (not all logs)
- Sent via `POST /api/v1/errors`
- TVM stores in centralized database

**Implementation:** See `tvm_client` package

---

## 8. Deployment Checklist

### 8.1 Pre-Deployment

- [ ] Hardware platform assembled (Tsuchiya's responsibility)
- [ ] Ubuntu 22.04 bootable USB prepared
- [ ] Network configuration planned (WiFi SSID, password)
- [ ] Vehicle ID assigned (e.g., VH-001, VH-002)
- [ ] TVM server URL and API key obtained (from Unno)

### 8.2 Deployment Steps

- [ ] Install Ubuntu 22.04 on vehicle compute unit
- [ ] Install system dependencies (ROS 2, build tools)
- [ ] Clone vehicle software repository
- [ ] Build vehicle software workspace
- [ ] Deploy configuration files (`/etc/multigo/config.yaml`)
- [ ] Deploy API key (`/etc/multigo/api_key.secret`)
- [ ] Install systemd service
- [ ] Enable and start service
- [ ] Verify service running (`systemctl status`)
- [ ] Verify ROS topics (`ros2 topic list`)
- [ ] Verify TVM connection (check logs for successful auth)

### 8.3 Post-Deployment

- [ ] Test basic motion (drive forward/backward)
- [ ] Test E-stop (hardware and software)
- [ ] Test TVM telemetry (verify data appears in TVM dashboard)
- [ ] Test navigation (simple waypoint navigation)
- [ ] Test docking (if docking station available)
- [ ] Document vehicle-specific configuration
- [ ] Add to vehicle inventory (spreadsheet or TVM database)

---

## 9. Troubleshooting

### 9.1 Common Issues

**Issue: Service fails to start**
```bash
# Check service status
sudo systemctl status multigo-vehicle

# Check logs
journalctl -u multigo-vehicle -n 100

# Common causes:
# - Missing dependencies (run rosdep install)
# - ROS setup not sourced (check ExecStart in service file)
# - Permission issues (service should run as 'multigo' user)
```

**Issue: Cannot connect to TVM server**
```bash
# Check network connectivity
ping tvm.example.com

# Check HTTPS connection
curl https://tvm.example.com/api/v1/health

# Check API key
cat /etc/multigo/api_key.secret  # Verify not empty

# Check logs for auth errors
journalctl -u multigo-vehicle | grep "TVM"
```

**Issue: ROS topics not publishing**
```bash
# Check if ROS nodes are running
ros2 node list

# Check specific topic
ros2 topic echo /sensors/lidar/points

# Check ROS domain ID
echo $ROS_DOMAIN_ID  # Should be 0

# Restart service
sudo systemctl restart multigo-vehicle
```

---

## 10. Comparison: Enterprise vs Pilot

**Removed from enterprise version:**

### Infrastructure
- ❌ Multi-region AWS deployment (us-east-1, us-west-2)
- ❌ Kubernetes orchestration (EKS)
- ❌ Load balancing (ALB, NLB)
- ❌ Auto-scaling groups
- ❌ VPC, subnets, security groups
- **Reason:** TVM server is Unno's responsibility, vehicle is single compute unit

### CI/CD
- ❌ Automated CI/CD pipeline (GitHub Actions for deployment)
- ❌ Blue-green deployments
- ❌ Canary releases
- ❌ Automated rollback
- **Reason:** Pilot uses manual deployment, CI/CD added later

### Monitoring
- ❌ Prometheus + Grafana
- ❌ ELK stack (Elasticsearch, Logstash, Kibana)
- ❌ CloudWatch dashboards
- ❌ PagerDuty alerts
- **Reason:** Pilot uses journalctl + TVM telemetry, advanced monitoring post-pilot

### Disaster Recovery
- ❌ Hot standby database in secondary region
- ❌ Automated failover
- ❌ RTO/RPO requirements
- ❌ Backup/restore procedures
- **Reason:** Not needed for pilot, server DR is Unno's responsibility

### Configuration Management
- ❌ AWS Systems Manager Parameter Store
- ❌ HashiCorp Vault
- ❌ Encrypted configuration at rest
- **Reason:** Pilot uses simple YAML files with file permissions

---

## Summary

**Simplified deployment for pilot:**
- ✅ Manual Ubuntu installation on vehicle
- ✅ ROS 2 Humble from apt packages
- ✅ Git clone + colcon build
- ✅ Systemd service for auto-start
- ✅ Simple YAML configuration
- ✅ Manual updates via SSH
- ✅ Local logging + TVM telemetry

**Total effort:** ~4 hours per vehicle for initial deployment

**Next Steps:**
1. Follow `DEVELOPMENT_SETUP_GUIDE.md` for development environment
2. Test deployment on development machine first
3. Deploy to vehicle after software testing complete
4. Document vehicle-specific configuration

---

**Document Status:** ✅ Active (Pilot Level)
**Next Review:** After first vehicle deployment
**Approved By:** Senior (December 19, 2025)

---

**END OF DOCUMENT**
