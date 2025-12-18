# Deployment Architecture

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Architecture Specification
**Status:** Active
**Version:** 1.0
**Last Updated:** 2025-12-17
**Owner:** Integration Team (Team 2 - TVM Infrastructure)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Deployment Overview](#2-deployment-overview)
3. [Infrastructure Architecture](#3-infrastructure-architecture)
4. [TVM Server Deployment](#4-tvm-server-deployment)
5. [Database Deployment](#5-database-deployment)
6. [Vehicle Software Deployment](#6-vehicle-software-deployment)
7. [CI/CD Pipeline](#7-cicd-pipeline)
8. [Environment Management](#8-environment-management)
9. [Infrastructure as Code](#9-infrastructure-as-code)
10. [Scaling and Load Balancing](#10-scaling-and-load-balancing)
11. [Rollout Strategies](#11-rollout-strategies)
12. [Configuration Management](#12-configuration-management)
13. [Monitoring and Observability Deployment](#13-monitoring-and-observability-deployment)
14. [Cost Optimization](#14-cost-optimization)

---

## 1. Executive Summary

### 1.1 Purpose

This document defines the deployment architecture for the outdoor wheelchair transport robot fleet system. It covers:
- **Infrastructure:** Cloud provider, regions, networking
- **TVM Server:** Containerization, orchestration, scaling
- **Vehicles:** Software deployment, OTA updates
- **CI/CD:** Automated build, test, deploy pipeline
- **Operations:** Monitoring, logging, configuration management

### 1.2 Deployment Strategy

**Cloud-First Approach:**
- **TVM Server:** AWS cloud (primary), on-premise option for air-gapped deployments
- **Vehicles:** Edge computing (onboard compute unit), autonomous operation capable
- **Hybrid:** Cloud for management, edge for operations

**Key Principles:**
1. **Infrastructure as Code:** All infrastructure versioned in Git (Terraform/CloudFormation)
2. **Containerization:** Docker for TVM server, consistent environments
3. **Automation:** CI/CD pipeline for zero-downtime deployments
4. **Multi-Region:** High availability across multiple AWS regions
5. **Cost-Effective:** Auto-scaling, spot instances, right-sizing

### 1.3 Technology Stack

| Component | Technology | Justification |
|-----------|------------|---------------|
| **Cloud Provider** | AWS | Industry standard, broad service offering, multi-region |
| **Container Runtime** | Docker | Standardized, portable, extensive tooling |
| **Orchestration** | Kubernetes (EKS) or ECS | Auto-scaling, self-healing, declarative |
| **CI/CD** | GitHub Actions | Integrated with code repo, flexible, cost-effective |
| **IaC** | Terraform | Multi-cloud, declarative, state management |
| **Configuration** | AWS Systems Manager Parameter Store | Secure, versioned, integrated |
| **Secrets** | HashiCorp Vault or AWS Secrets Manager | Secure, audited, rotation |
| **Monitoring** | Prometheus + Grafana | Open-source, flexible, Kubernetes-native |
| **Logging** | ELK Stack (Elasticsearch, Logstash, Kibana) | Centralized, searchable, visualization |

---

## 2. Deployment Overview

### 2.1 Deployment Topology

```
┌─────────────────────────────────────────────────────────────┐
│                    AWS Cloud (Primary Region)               │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  VPC (10.0.0.0/16)                                    │  │
│  │                                                       │  │
│  │  ┌─────────────────┐  ┌─────────────────┐           │  │
│  │  │ Public Subnets  │  │ Private Subnets │           │  │
│  │  │ (DMZ)           │  │ (App + Data)    │           │  │
│  │  ├─────────────────┤  ├─────────────────┤           │  │
│  │  │ - Load Balancer │  │ - TVM Servers   │           │  │
│  │  │ - NAT Gateway   │  │ - Database      │           │  │
│  │  │ - Bastion Host  │  │ - Redis Cache   │           │  │
│  │  └─────────────────┘  └─────────────────┘           │  │
│  │                                                       │  │
│  │  Availability Zones: us-east-1a, us-east-1b, us-east-1c │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              AWS Cloud (Secondary Region - DR)              │
│  - Hot standby database (us-west-2)                         │
│  - Minimal compute (1 instance, scaled up on failover)      │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                     On-Premise (Facility)                   │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ Vehicle Fleet                                         │  │
│  │  - Vehicle 1 (Ubuntu 22.04 + ROS 2 Humble)           │  │
│  │  - Vehicle 2                                          │  │
│  │  - Vehicle 3                                          │  │
│  │  ...                                                  │  │
│  │                                                       │  │
│  │ WiFi Access Points (connect vehicles to internet)    │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Deployment Phases

**Phase 1: MVP Deployment (Weeks 1-18)**
- Single AWS region (us-east-1)
- Manual deployments (Docker Compose or simple ECS)
- Single environment (production)
- 1-3 vehicles

**Phase 2: Production Deployment (Weeks 19-24)**
- Multi-region (us-east-1 + us-west-2 DR)
- Automated CI/CD pipeline
- Multiple environments (dev, staging, production)
- 5-10 vehicles
- Kubernetes (EKS) or ECS with auto-scaling

**Phase 3: Scale-Out (Months 7-12)**
- Multi-facility support
- 50+ vehicles
- Advanced monitoring and alerting
- Cost optimization (spot instances, reserved instances)

---

## 3. Infrastructure Architecture

### 3.1 AWS Account Structure

**Multi-Account Strategy (AWS Organizations):**

```
Root Organization Account
├── Production Account (prod.tvm.example.com)
│   ├── VPC: 10.0.0.0/16
│   ├── TVM Server (EKS cluster)
│   ├── RDS PostgreSQL (multi-AZ)
│   └── S3 Buckets (backups, logs)
├── Staging Account (staging.tvm.example.com)
│   ├── VPC: 10.1.0.0/16
│   ├── TVM Server (smaller instance types)
│   └── RDS PostgreSQL (single-AZ, cost-effective)
├── Development Account (dev.tvm.example.com)
│   ├── VPC: 10.2.0.0/16
│   └── Shared resources (cost-effective)
└── Security/Audit Account
    ├── CloudTrail logs
    ├── GuardDuty
    └── Security Hub
```

**Benefits:**
- **Isolation:** Production changes don't affect staging/dev
- **Security:** Least privilege per account
- **Cost Management:** Track costs per environment

### 3.2 Network Architecture

**VPC Design (Production Account):**

```
VPC: 10.0.0.0/16
├── Public Subnets (DMZ)
│   ├── 10.0.1.0/24 (us-east-1a)
│   ├── 10.0.2.0/24 (us-east-1b)
│   └── 10.0.3.0/24 (us-east-1c)
│   - Internet Gateway attached
│   - Contains: Load Balancer, NAT Gateway, Bastion Host
├── Private Subnets (Application Tier)
│   ├── 10.0.11.0/24 (us-east-1a)
│   ├── 10.0.12.0/24 (us-east-1b)
│   └── 10.0.13.0/24 (us-east-1c)
│   - Route to internet via NAT Gateway
│   - Contains: TVM Server (EKS nodes), Redis, Vault
└── Private Subnets (Data Tier)
    ├── 10.0.21.0/24 (us-east-1a)
    ├── 10.0.22.0/24 (us-east-1b)
    └── 10.0.23.0/24 (us-east-1c)
    - No internet access
    - Contains: RDS PostgreSQL, ElastiCache Redis
```

**Routing:**
- **Public Subnets:** IGW → Internet (0.0.0.0/0)
- **Private Subnets (App):** NAT Gateway → Internet (for outbound only)
- **Private Subnets (Data):** No internet route (isolated)

### 3.3 Security Groups

| Security Group | Inbound Rules | Outbound Rules | Purpose |
|----------------|---------------|----------------|---------|
| **ALB-SG** | 443 from 0.0.0.0/0 | All to TVM-SG | Load balancer |
| **TVM-SG** | 8080 from ALB-SG | All to DB-SG, Redis-SG | TVM servers |
| **DB-SG** | 5432 from TVM-SG | None | PostgreSQL |
| **Redis-SG** | 6379 from TVM-SG | None | Redis cache |
| **Bastion-SG** | 22 from Admin IPs only | All (for SSH tunneling) | Admin access |

**Principle:** Least privilege, deny by default.

---

## 4. TVM Server Deployment

### 4.1 Containerization

**Docker Image:**

```dockerfile
# Dockerfile for TVM Server (Node.js example)
FROM node:18-alpine AS build
WORKDIR /app
COPY package*.json ./
RUN npm ci --only=production
COPY . .
RUN npm run build

FROM node:18-alpine AS runtime
WORKDIR /app
COPY --from=build /app/dist ./dist
COPY --from=build /app/node_modules ./node_modules
EXPOSE 8080
USER node
CMD ["node", "dist/server.js"]
```

**Image Tagging:**
- **Pattern:** `<registry>/<image>:<version>-<commit-sha>`
- **Example:** `123456789012.dkr.ecr.us-east-1.amazonaws.com/tvm-server:v1.2.3-abc1234`
- **Tags:** `latest` (not used in production), `v1.2.3`, `v1.2.3-abc1234`

**Registry:**
- **AWS ECR (Elastic Container Registry):** Private Docker registry
- **Image Scanning:** Automatic vulnerability scanning on push
- **Lifecycle Policy:** Retain last 10 versions, delete older (cost optimization)

### 4.2 Orchestration (Kubernetes - EKS)

**EKS Cluster Configuration:**

```yaml
# EKS Cluster (Terraform)
resource "aws_eks_cluster" "tvm" {
  name     = "tvm-production"
  role_arn = aws_iam_role.eks_cluster.arn
  version  = "1.28"

  vpc_config {
    subnet_ids = [
      aws_subnet.private_1a.id,
      aws_subnet.private_1b.id,
      aws_subnet.private_1c.id
    ]
    endpoint_private_access = true
    endpoint_public_access  = false  # Secure: Only private access
  }
}

resource "aws_eks_node_group" "tvm" {
  cluster_name    = aws_eks_cluster.tvm.name
  node_group_name = "tvm-nodes"
  node_role_arn   = aws_iam_role.eks_node.arn
  subnet_ids      = [
    aws_subnet.private_1a.id,
    aws_subnet.private_1b.id,
    aws_subnet.private_1c.id
  ]

  scaling_config {
    desired_size = 3
    max_size     = 10
    min_size     = 2
  }

  instance_types = ["t3.large"]  # 2 vCPU, 8 GB RAM
}
```

**Kubernetes Deployment:**

```yaml
# deployment.yaml (TVM Server)
apiVersion: apps/v1
kind: Deployment
metadata:
  name: tvm-server
  namespace: production
spec:
  replicas: 3
  selector:
    matchLabels:
      app: tvm-server
  template:
    metadata:
      labels:
        app: tvm-server
        version: v1.2.3
    spec:
      containers:
      - name: tvm-server
        image: 123456789012.dkr.ecr.us-east-1.amazonaws.com/tvm-server:v1.2.3-abc1234
        ports:
        - containerPort: 8080
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: tvm-secrets
              key: database_url
        - name: REDIS_URL
          valueFrom:
            configMapKeyRef:
              name: tvm-config
              key: redis_url
        resources:
          requests:
            cpu: "500m"
            memory: "1Gi"
          limits:
            cpu: "1000m"
            memory: "2Gi"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8080
          initialDelaySeconds: 10
          periodSeconds: 5
```

**Kubernetes Service (Load Balancer):**

```yaml
# service.yaml
apiVersion: v1
kind: Service
metadata:
  name: tvm-server
  namespace: production
  annotations:
    service.beta.kubernetes.io/aws-load-balancer-type: "nlb"  # Network Load Balancer
spec:
  type: LoadBalancer
  selector:
    app: tvm-server
  ports:
  - protocol: TCP
    port: 443
    targetPort: 8080
```

### 4.3 Alternative: ECS Deployment

**If NOT using Kubernetes, use ECS (simpler, less operational overhead):**

```json
// ECS Task Definition
{
  "family": "tvm-server",
  "networkMode": "awsvpc",
  "requiresCompatibilities": ["FARGATE"],
  "cpu": "512",
  "memory": "1024",
  "containerDefinitions": [
    {
      "name": "tvm-server",
      "image": "123456789012.dkr.ecr.us-east-1.amazonaws.com/tvm-server:v1.2.3",
      "portMappings": [{"containerPort": 8080}],
      "environment": [
        {"name": "NODE_ENV", "value": "production"}
      ],
      "secrets": [
        {
          "name": "DATABASE_URL",
          "valueFrom": "arn:aws:secretsmanager:us-east-1:123456789012:secret:database-url"
        }
      ],
      "logConfiguration": {
        "logDriver": "awslogs",
        "options": {
          "awslogs-group": "/ecs/tvm-server",
          "awslogs-region": "us-east-1",
          "awslogs-stream-prefix": "ecs"
        }
      }
    }
  ]
}
```

**ECS Service (with ALB):**
- **Desired Count:** 3 tasks
- **Launch Type:** Fargate (serverless, no EC2 management)
- **Load Balancer:** Application Load Balancer (ALB)
- **Auto Scaling:** Target tracking (CPU 70%, scale up/down)

**EKS vs. ECS:**
| Feature | EKS (Kubernetes) | ECS (Fargate) |
|---------|------------------|---------------|
| **Complexity** | High (K8s learning curve) | Low (AWS-native) |
| **Flexibility** | Very high (cloud-agnostic) | Medium (AWS-only) |
| **Cost** | $0.10/hour cluster + EC2 | Pay per task (no cluster cost) |
| **Recommendation** | If multi-cloud or K8s expertise | If AWS-only and simplicity valued |

**Decision:** Start with **ECS Fargate** (simpler), migrate to EKS if multi-cloud needed.

---

## 5. Database Deployment

### 5.1 RDS PostgreSQL

**Configuration:**

```hcl
# Terraform: RDS PostgreSQL
resource "aws_db_instance" "tvm_primary" {
  identifier             = "tvm-production-primary"
  engine                 = "postgres"
  engine_version         = "15.4"
  instance_class         = "db.t3.large"  # 2 vCPU, 8 GB RAM
  allocated_storage      = 100  # GB
  max_allocated_storage  = 1000  # Auto-scaling up to 1 TB
  storage_type           = "gp3"  # General Purpose SSD
  storage_encrypted      = true
  kms_key_id             = aws_kms_key.rds.arn

  db_name  = "tvm"
  username = "tvm_admin"
  password = random_password.db_password.result  # Generated, stored in Secrets Manager

  multi_az               = true  # High availability (standby in different AZ)
  publicly_accessible    = false  # Private subnet only
  vpc_security_group_ids = [aws_security_group.db.id]
  db_subnet_group_name   = aws_db_subnet_group.tvm.name

  backup_retention_period = 7  # Days
  backup_window           = "03:00-04:00"  # UTC
  maintenance_window      = "sun:04:00-sun:05:00"  # UTC

  enabled_cloudwatch_logs_exports = ["postgresql", "upgrade"]

  deletion_protection = true  # Prevent accidental deletion

  tags = {
    Environment = "production"
  }
}
```

**Read Replica (if needed for read-heavy workload):**

```hcl
resource "aws_db_instance" "tvm_read_replica" {
  identifier              = "tvm-production-replica"
  replicate_source_db     = aws_db_instance.tvm_primary.id
  instance_class          = "db.t3.medium"  # Smaller than primary (cost optimization)
  publicly_accessible     = false
  vpc_security_group_ids  = [aws_security_group.db.id]

  tags = {
    Environment = "production"
    Type        = "read-replica"
  }
}
```

### 5.2 Database Migrations

**Migration Tool:** Flyway or Liquibase (or native ORM migrations)

**Migration Files:**
```
migrations/
├── V001__create_residents_table.sql
├── V002__create_vehicles_table.sql
├── V003__create_missions_table.sql
└── V004__add_mission_priority.sql
```

**Deployment Process:**
1. **Build:** Migrations bundled with application
2. **Deploy:** On application startup, run migrations automatically
3. **Rollback:** Flyway supports rollback (if migration fails, don't deploy app)

**Best Practices:**
- **Backward Compatible:** Migrations should not break existing code (e.g., add column with default value)
- **Tested:** Migrations tested in staging before production
- **Idempotent:** Safe to run multiple times (use `IF NOT EXISTS`)

---

## 6. Vehicle Software Deployment

### 6.1 Vehicle Software Stack

```
Vehicle Compute Unit (AMD Ryzen 7 7840HS, 32 GB RAM, 512 GB SSD)
├── Operating System: Ubuntu 22.04 LTS
├── ROS 2 Humble (installed via apt)
├── Vehicle Software (custom packages)
│   ├── nav_control (navigation)
│   ├── nav_docking (docking)
│   ├── swerve_drive_controller (kinematics)
│   ├── pcl_merge (sensor fusion)
│   └── tvm_bridge (TVM communication)
├── System Services
│   ├── Systemd (service management)
│   ├── NetworkManager (WiFi management)
│   └── Chrony (time synchronization)
└── Monitoring Agents
    ├── Prometheus Node Exporter (system metrics)
    └── Filebeat (log shipping to ELK)
```

### 6.2 Initial Provisioning

**Procedure: New Vehicle Setup**

1. **Base OS Installation:**
   - Flash Ubuntu 22.04 to SSD (using Ventoy or dd)
   - Boot vehicle, complete initial setup (user account, hostname)
   - Hostname: `vehicle-<ID>` (e.g., `vehicle-001`)

2. **Ansible Provisioning:**
   ```bash
   # Ansible playbook: provision_vehicle.yml
   ansible-playbook -i vehicle-001.local, provision_vehicle.yml
   ```

   Playbook installs:
   - ROS 2 Humble
   - Vehicle software dependencies (PCL, OpenCV, etc.)
   - System configuration (network, time sync, firewall)
   - Monitoring agents

3. **Vehicle Registration:**
   - Generate vehicle identity (UUID)
   - Generate mTLS certificate (signed by internal CA)
   - Store certificate in TPM 2.0 (if available)
   - Register vehicle in TVM database: `INSERT INTO vehicles (id, name, status) VALUES (...)`

4. **Deploy Vehicle Software:**
   - Clone vehicle software repo: `git clone https://github.com/yourorg/vehicle-software.git`
   - Build: `colcon build`
   - Install systemd service: `systemctl enable vehicle-software`
   - Start: `systemctl start vehicle-software`

**Total Time:** 2-3 hours per vehicle

### 6.3 OTA (Over-The-Air) Updates

**OTA Update Architecture:**

```
┌─────────────────────────────────────────────────────────┐
│ 1. New Vehicle Software Release                        │
│    - Developer commits code to Git                      │
│    - CI/CD builds Debian package (.deb)                 │
│    - Package signed with GPG key                        │
│    - Uploaded to S3 bucket (tvm-ota-updates/)          │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 2. TVM Server Notifies Vehicles                        │
│    - TVM pushes notification to vehicles via MQTT       │
│    - Message: "Update available: v1.2.3"                │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 3. Vehicle Downloads Update                            │
│    - Vehicle downloads .deb from S3 (over TLS)          │
│    - Verify GPG signature (reject if invalid)           │
│    - Verify checksum (SHA-256)                          │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 4. Vehicle Installs Update (A/B Partitioning)          │
│    - Install to staging partition (Partition B)         │
│    - Update bootloader: Boot from Partition B next      │
│    - Reboot vehicle                                     │
└────────────────────┬────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────┐
│ 5. Health Check                                         │
│    - Vehicle boots into Partition B (new software)      │
│    - Run health check: ROS nodes running? Sensors OK?   │
│    - If health check passes → Mark Partition B stable   │
│    - If health check fails 3x → Rollback to Partition A │
└─────────────────────────────────────────────────────────┘
```

**A/B Partitioning:**
- **Partition A:** Current stable version (v1.2.2)
- **Partition B:** New version (v1.2.3)
- **Rollback:** If new version fails, reboot into Partition A

**Implementation:** Use `rauc` (Robust Auto-Update Controller) or custom script

**OTA Update Schedule:**
- **Frequency:** Every 2 weeks (or as needed for critical fixes)
- **Rollout:** Phased (1 vehicle → 10% of fleet → 100%)
- **Downtime:** Vehicle unavailable for 5-10 minutes during reboot

### 6.4 Configuration Management (Vehicles)

**Vehicle Configuration:**
- **Static Config:** Baked into OS image (ROS parameters, network settings)
- **Dynamic Config:** Fetched from TVM at startup (maps, docking positions)

**Map Updates:**
- Maps stored on vehicle SSD: `/opt/vehicle/maps/facility_map.pgm`
- Updated via OTA: New map downloaded from S3, replaces old map
- Trigger re-localization after map update (vehicle re-scans environment)

---

## 7. CI/CD Pipeline

### 7.1 CI/CD Workflow (TVM Server)

**GitHub Actions Pipeline:**

```yaml
# .github/workflows/deploy.yml
name: Deploy TVM Server

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
      - run: npm ci
      - run: npm test  # Unit tests
      - run: npm run lint  # ESLint
      - run: npm run build  # TypeScript compilation

  build:
    runs-on: ubuntu-latest
    needs: test
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v3
      - name: Configure AWS credentials
        uses: aws-actions/configure-aws-credentials@v2
        with:
          role-to-assume: arn:aws:iam::123456789012:role/GitHubActionsRole
          aws-region: us-east-1
      - name: Login to Amazon ECR
        id: login-ecr
        uses: aws-actions/amazon-ecr-login@v1
      - name: Build, tag, and push image to Amazon ECR
        env:
          ECR_REGISTRY: ${{ steps.login-ecr.outputs.registry }}
          ECR_REPOSITORY: tvm-server
          IMAGE_TAG: ${{ github.sha }}
        run: |
          docker build -t $ECR_REGISTRY/$ECR_REPOSITORY:$IMAGE_TAG .
          docker push $ECR_REGISTRY/$ECR_REPOSITORY:$IMAGE_TAG
          docker tag $ECR_REGISTRY/$ECR_REPOSITORY:$IMAGE_TAG $ECR_REGISTRY/$ECR_REPOSITORY:latest
          docker push $ECR_REGISTRY/$ECR_REPOSITORY:latest

  deploy-staging:
    runs-on: ubuntu-latest
    needs: build
    environment: staging
    steps:
      - name: Deploy to ECS Staging
        run: |
          aws ecs update-service --cluster tvm-staging --service tvm-server --force-new-deployment

  integration-test:
    runs-on: ubuntu-latest
    needs: deploy-staging
    steps:
      - run: npm run test:integration  # Run integration tests against staging

  deploy-production:
    runs-on: ubuntu-latest
    needs: integration-test
    environment: production
    steps:
      - name: Deploy to ECS Production
        run: |
          aws ecs update-service --cluster tvm-production --service tvm-server --force-new-deployment --desired-count 3
```

**Pipeline Stages:**
1. **Test:** Unit tests, linting, build (on every PR)
2. **Build:** Build Docker image, push to ECR (on merge to `main`)
3. **Deploy Staging:** Deploy to staging environment automatically
4. **Integration Test:** Run integration tests against staging
5. **Deploy Production:** Manual approval required (GitHub Environment protection rule)

**Deployment Time:** ~10 minutes (test → build → deploy)

### 7.2 CI/CD Workflow (Vehicle Software)

**GitHub Actions Pipeline (Vehicle):**

```yaml
# .github/workflows/vehicle-deploy.yml
name: Deploy Vehicle Software

on:
  push:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Install ROS 2 Humble
        run: |
          sudo apt update
          sudo apt install -y ros-humble-desktop
      - name: Build ROS 2 packages
        run: |
          source /opt/ros/humble/setup.bash
          colcon build
      - name: Run ROS 2 tests
        run: |
          source /opt/ros/humble/setup.bash
          colcon test
          colcon test-result --verbose

  build-deb:
    runs-on: ubuntu-22.04
    needs: test
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v3
      - name: Build Debian package
        run: |
          # Package vehicle software as .deb
          dpkg-buildpackage -us -uc
      - name: Sign package
        run: |
          dpkg-sig --sign builder vehicle-software_1.2.3_amd64.deb
      - name: Upload to S3
        run: |
          aws s3 cp vehicle-software_1.2.3_amd64.deb s3://tvm-ota-updates/
      - name: Notify TVM
        run: |
          curl -X POST https://api.tvm.example.com/api/ota/new-release \
            -H "Authorization: Bearer ${{ secrets.TVM_API_KEY }}" \
            -d '{"version": "1.2.3", "url": "https://tvm-ota-updates.s3.amazonaws.com/vehicle-software_1.2.3_amd64.deb"}'
```

**Rollout Strategy:**
- **Canary:** Deploy to 1 vehicle first (8 hours observation)
- **Gradual:** 10% → 50% → 100% (over 3 days)
- **Rollback:** If errors >5% on canary, rollback all

---

## 8. Environment Management

### 8.1 Environment Definitions

| Environment | Purpose | Infrastructure | Cost | Uptime |
|-------------|---------|----------------|------|--------|
| **Development** | Developer testing, rapid iteration | Shared AWS account, small instances | $50/month | Best-effort (nights/weekends off) |
| **Staging** | Pre-production testing, QA | Dedicated AWS account, production-like | $500/month | 99% (maintenance windows) |
| **Production** | Live system, real users | Dedicated AWS account, multi-AZ | $2000/month | 99.5% SLA |

### 8.2 Environment Parity

**Goal:** Staging should mirror production as closely as possible

**Differences (Acceptable):**
- **Instance Count:** Staging: 1 TVM server, Production: 3+ TVM servers
- **Instance Size:** Staging: t3.medium, Production: t3.large
- **Database:** Staging: Single-AZ, Production: Multi-AZ

**Same:**
- **Software Version:** Staging deployed first, production uses same image
- **Configuration:** Same environment variables (different values, e.g., DATABASE_URL)
- **Dependencies:** Same RDS version, same Redis version

### 8.3 Environment Promotion

**Deployment Flow:**
```
Developer Laptop
   ↓ (git push)
GitHub (main branch)
   ↓ (CI/CD build)
ECR (Docker image)
   ↓ (automatic)
Staging Environment
   ↓ (integration tests pass + manual approval)
Production Environment
```

**Manual Approval Required:** Production deployment requires approval from:
- **Unno-san** (TVM Lead) or **Pankaj** (Vehicle Lead)

---

## 9. Infrastructure as Code

### 9.1 Terraform Structure

```
terraform/
├── modules/
│   ├── vpc/              # Reusable VPC module
│   │   ├── main.tf
│   │   ├── variables.tf
│   │   └── outputs.tf
│   ├── ecs/              # ECS cluster module
│   ├── rds/              # RDS PostgreSQL module
│   └── eks/              # EKS cluster module (if using K8s)
├── environments/
│   ├── dev/
│   │   ├── main.tf       # Calls modules with dev-specific variables
│   │   ├── variables.tf
│   │   └── terraform.tfvars
│   ├── staging/
│   │   ├── main.tf
│   │   └── terraform.tfvars
│   └── production/
│       ├── main.tf
│       └── terraform.tfvars
└── global/
    ├── iam/              # IAM roles, policies (shared across environments)
    └── s3/               # S3 buckets (backups, logs)
```

### 9.2 Terraform Workflow

**Commands:**
```bash
# Initialize Terraform
cd terraform/environments/production
terraform init

# Plan changes (dry-run)
terraform plan -out=tfplan

# Review plan (manual review)
cat tfplan

# Apply changes
terraform apply tfplan

# Verify infrastructure
terraform show
```

**State Management:**
- **Backend:** S3 bucket (`tvm-terraform-state`) + DynamoDB table (state locking)
- **State File:** `s3://tvm-terraform-state/production/terraform.tfstate`
- **Locking:** Prevents concurrent `terraform apply` (DynamoDB lock table)

**Example Terraform Backend:**
```hcl
terraform {
  backend "s3" {
    bucket         = "tvm-terraform-state"
    key            = "production/terraform.tfstate"
    region         = "us-east-1"
    dynamodb_table = "terraform-locks"
    encrypt        = true
  }
}
```

### 9.3 Infrastructure Changes

**Process:**
1. **Developer:** Modify Terraform code (e.g., increase RDS instance size)
2. **PR:** Create pull request with Terraform changes
3. **CI:** GitHub Actions runs `terraform plan`, posts plan to PR comments
4. **Review:** Team reviews plan (cost impact, blast radius)
5. **Merge:** PR merged to `main`
6. **Deploy:** GitHub Actions runs `terraform apply` (manual approval for production)

---

## 10. Scaling and Load Balancing

### 10.1 Auto-Scaling (TVM Server)

**ECS Auto-Scaling Policy (Target Tracking):**

```hcl
resource "aws_appautoscaling_target" "tvm_server" {
  max_capacity       = 10
  min_capacity       = 2
  resource_id        = "service/tvm-production/tvm-server"
  scalable_dimension = "ecs:service:DesiredCount"
  service_namespace  = "ecs"
}

resource "aws_appautoscaling_policy" "tvm_server_cpu" {
  name               = "tvm-server-cpu-scaling"
  policy_type        = "TargetTrackingScaling"
  resource_id        = aws_appautoscaling_target.tvm_server.resource_id
  scalable_dimension = aws_appautoscaling_target.tvm_server.scalable_dimension
  service_namespace  = aws_appautoscaling_target.tvm_server.service_namespace

  target_tracking_scaling_policy_configuration {
    predefined_metric_specification {
      predefined_metric_type = "ECSServiceAverageCPUUtilization"
    }
    target_value = 70.0  # Scale up if CPU > 70%
  }
}
```

**Scaling Behavior:**
- **Scale Up:** If CPU > 70% for 3 minutes → Add 1 task
- **Scale Down:** If CPU < 50% for 10 minutes → Remove 1 task
- **Cooldown:** 5 minutes between scaling actions

**Load Testing:** Use `k6` or `Locust` to simulate 1000 concurrent users, verify auto-scaling works

### 10.2 Database Scaling

**Vertical Scaling (RDS):**
- **Manual:** Change instance class (db.t3.large → db.m5.xlarge)
- **Downtime:** 5-10 minutes (RDS restarts)
- **When:** CPU consistently >80%, or IOPS maxed out

**Horizontal Scaling (Read Replicas):**
- **Read Replica:** Offload read queries to replica (reports, analytics)
- **Application Change:** Direct reads to replica endpoint
- **When:** Read:Write ratio > 3:1

**Connection Pooling:**
- **PgBouncer:** Deploy as sidecar container, pool connections
- **Benefit:** Handle 1000+ client connections with only 50 DB connections

### 10.3 Load Balancing

**Application Load Balancer (ALB):**
- **Algorithm:** Round-robin (default)
- **Health Check:** HTTP GET `/health` every 30 seconds
- **Unhealthy Threshold:** 2 consecutive failures → Remove from pool
- **Healthy Threshold:** 3 consecutive successes → Add back to pool

**Sticky Sessions (If Needed):**
- **Use Case:** If TVM server maintains session state (not recommended, use Redis instead)
- **Implementation:** ALB cookie-based stickiness (AWSALB cookie)

---

## 11. Rollout Strategies

### 11.1 Blue-Green Deployment

**Strategy:** Run two identical production environments (Blue and Green), switch traffic instantly

```
┌────────────────────────────────────────────────────────┐
│               Load Balancer (Route 53)                 │
│  DNS: api.tvm.example.com                              │
└─────────────┬───────────────────────────┬──────────────┘
              │                           │
       (100% traffic)             (0% traffic)
              ↓                           ↓
     ┌─────────────────┐         ┌─────────────────┐
     │ Blue Environment│         │Green Environment│
     │ (Current: v1.2.2)│         │ (New: v1.2.3)   │
     │ - 3 TVM servers │         │ - 3 TVM servers │
     └─────────────────┘         └─────────────────┘

# After verification of Green:
# Switch DNS: 0% → 100% to Green
# Blue becomes standby (or terminated)
```

**Procedure:**
1. **Deploy Green:** Deploy new version (v1.2.3) to Green environment
2. **Test Green:** Run smoke tests, health checks
3. **Switch Traffic:** Route 53 weighted routing: 100% → Green
4. **Monitor:** Watch error rates, latency for 1 hour
5. **Rollback (If Needed):** Switch back to Blue (5 minutes)
6. **Decommission Blue:** After 24 hours, terminate Blue environment (cost savings)

**Pros:** Zero downtime, instant rollback
**Cons:** Double cost during deployment (2 environments running)

### 11.2 Canary Deployment

**Strategy:** Roll out new version to small subset (10%) of users first

```
Load Balancer
├── 90% traffic → Old version (v1.2.2) - 9 tasks
└── 10% traffic → New version (v1.2.3) - 1 task

# If canary healthy after 1 hour:
# Gradually increase: 10% → 50% → 100%
```

**Procedure:**
1. **Deploy Canary:** Deploy 1 task with new version
2. **Route 10% Traffic:** ALB weighted target groups (90:10)
3. **Monitor Canary:** Error rate, latency, custom metrics
4. **Gradual Rollout:** Every 1 hour, increase traffic (10% → 50% → 100%)
5. **Rollback (If Needed):** Remove canary task, route 100% to old version

**Pros:** Reduce blast radius, gradual rollout
**Cons:** More complex, requires traffic splitting

### 11.3 Rolling Deployment

**Strategy:** Replace instances one-by-one (default ECS/K8s behavior)

```
Current: [v1.2.2] [v1.2.2] [v1.2.2]
Step 1:  [v1.2.3] [v1.2.2] [v1.2.2]  # Replace 1st instance
Step 2:  [v1.2.3] [v1.2.3] [v1.2.2]  # Replace 2nd instance
Step 3:  [v1.2.3] [v1.2.3] [v1.2.3]  # Replace 3rd instance
```

**Procedure:**
1. **Start Deployment:** ECS: `aws ecs update-service --force-new-deployment`
2. **Rolling Update:** ECS stops 1 task (v1.2.2), starts 1 task (v1.2.3)
3. **Health Check:** Wait for new task to pass health check
4. **Repeat:** Continue until all tasks updated

**Pros:** Simple, built-in to ECS/K8s
**Cons:** Gradual rollout (10-15 minutes), both versions running simultaneously

**Recommendation:** Use **Rolling Deployment** (default) for most deployments, **Canary** for high-risk changes.

---

## 12. Configuration Management

### 12.1 Configuration Storage

**AWS Systems Manager Parameter Store:**

```bash
# Store configuration parameter
aws ssm put-parameter \
  --name "/tvm/production/database-url" \
  --value "postgresql://user:pass@db.example.com:5432/tvm" \
  --type "SecureString" \
  --kms-key-id "arn:aws:kms:us-east-1:123456789012:key/abc123"

# Retrieve parameter
aws ssm get-parameter --name "/tvm/production/database-url" --with-decryption
```

**Hierarchy:**
```
/tvm/
├── production/
│   ├── database-url (SecureString)
│   ├── redis-url (String)
│   ├── jwt-secret (SecureString)
│   └── feature-flags/
│       ├── enable-ai-docking (String: "true")
│       └── max-concurrent-missions (String: "50")
├── staging/
│   ├── database-url
│   └── ...
└── development/
    └── ...
```

**Access Control:** IAM policy restricts access per environment
- TVM production servers can only read `/tvm/production/*`
- TVM staging servers can only read `/tvm/staging/*`

### 12.2 Secrets Management

**AWS Secrets Manager (for Database Passwords):**

```bash
# Create secret
aws secretsmanager create-secret \
  --name "tvm/production/db-password" \
  --secret-string "super-secure-password-xyz123" \
  --kms-key-id "arn:aws:kms:us-east-1:123456789012:key/abc123"

# Enable automatic rotation (every 30 days)
aws secretsmanager rotate-secret \
  --secret-id "tvm/production/db-password" \
  --rotation-lambda-arn "arn:aws:lambda:us-east-1:123456789012:function:SecretsManagerRDSRotation" \
  --rotation-rules AutomaticallyAfterDays=30
```

**Secrets in Application:**
- **ECS:** Inject secrets as environment variables from Secrets Manager
- **Lambda:** Use AWS SDK to fetch secrets at runtime

**HashiCorp Vault (Alternative):**
- For more complex use cases (dynamic secrets, PKI)
- Requires operational overhead (Vault cluster management)

### 12.3 Feature Flags

**Purpose:** Enable/disable features without code deployment

**Implementation:**
- **Storage:** Parameter Store or LaunchDarkly (SaaS)
- **Example:** `/tvm/production/feature-flags/enable-ai-docking = "true"`

**Usage in Code:**
```javascript
const enableAIDocking = await getParameter('/tvm/production/feature-flags/enable-ai-docking');
if (enableAIDocking === 'true') {
  // Use AI-based docking
} else {
  // Use traditional ArUco marker docking
}
```

**Benefits:** A/B testing, gradual rollouts, instant rollback (flip flag)

---

## 13. Monitoring and Observability Deployment

### 13.1 Monitoring Stack

**Prometheus + Grafana (Kubernetes):**

```yaml
# Helm install Prometheus + Grafana
helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
helm install prometheus prometheus-community/kube-prometheus-stack -n monitoring

# Access Grafana
kubectl port-forward -n monitoring svc/prometheus-grafana 3000:80
# Open http://localhost:3000 (admin/prom-operator)
```

**CloudWatch (ECS):**
- **Metrics:** ECS publishes CPU, memory, network metrics automatically
- **Alarms:** Create CloudWatch Alarms (CPU > 80% for 5 min → SNS notification)
- **Dashboard:** CloudWatch Dashboard for at-a-glance view

### 13.2 Logging Stack

**ELK Stack Deployment (AWS Managed Elasticsearch):**

```hcl
resource "aws_elasticsearch_domain" "tvm_logs" {
  domain_name           = "tvm-logs-production"
  elasticsearch_version = "7.10"

  cluster_config {
    instance_type            = "t3.medium.elasticsearch"
    instance_count           = 3
    zone_awareness_enabled   = true
    zone_awareness_config {
      availability_zone_count = 3
    }
  }

  ebs_options {
    ebs_enabled = true
    volume_size = 100  # GB per node
  }

  vpc_options {
    subnet_ids = [
      aws_subnet.private_1a.id,
      aws_subnet.private_1b.id,
      aws_subnet.private_1c.id
    ]
    security_group_ids = [aws_security_group.elasticsearch.id]
  }

  encrypt_at_rest {
    enabled = true
  }

  node_to_node_encryption {
    enabled = true
  }
}
```

**Filebeat Configuration (on TVM servers):**
```yaml
# filebeat.yml
filebeat.inputs:
- type: container
  paths:
    - /var/log/containers/*.log
output.elasticsearch:
  hosts: ["tvm-logs-production.es.amazonaws.com:443"]
  protocol: "https"
  username: "elastic"
  password: "${ELASTICSEARCH_PASSWORD}"
```

**Kibana:**
- Deployed as part of AWS Managed Elasticsearch
- Access: `https://tvm-logs-production.es.amazonaws.com/_plugin/kibana`

---

## 14. Cost Optimization

### 14.1 Cost Breakdown (Estimated Monthly)

| Resource | Configuration | Cost/month | Optimization |
|----------|---------------|------------|--------------|
| **ECS Fargate** | 3 tasks, 0.5 vCPU, 1GB RAM, 24/7 | $36 | Use Spot Fargate (50% savings) |
| **RDS PostgreSQL** | db.t3.large, 100GB, multi-AZ | $200 | Use reserved instance (40% savings) |
| **ElastiCache Redis** | cache.t3.medium | $50 | Right-size based on usage |
| **ALB** | 1 ALB, 10GB/month | $25 | Share ALB across services |
| **NAT Gateway** | 3 NAT gateways, 100GB/month | $135 | Use 1 NAT gateway (compromise availability) |
| **Data Transfer** | 500GB outbound | $45 | CloudFront CDN for static assets |
| **CloudWatch Logs** | 50GB ingestion, 90-day retention | $30 | Reduce retention to 30 days |
| **S3 Backups** | 1TB storage (Standard), 100GB/month transfer | $30 | Lifecycle policy → Glacier after 90 days |
| **Route 53** | 2 hosted zones, 1M queries/month | $2 | Negligible |
| **EKS Control Plane** | (If using EKS) | $72 | Use ECS instead (no cluster cost) |
| **Total (ECS)** | | **~$550/month** | |
| **Total (EKS)** | | **~$620/month** | |

**Production (Optimized):** ~$400/month (with reserved instances, Spot, 1 NAT gateway)

### 14.2 Cost Optimization Strategies

**1. Right-Sizing:**
- **Monitor:** Use CloudWatch metrics to identify over-provisioned instances
- **Action:** Downsize instances (e.g., db.t3.large → db.t3.medium if CPU <30%)

**2. Reserved Instances:**
- **RDS:** 1-year reserved instance (40% savings)
- **Commitment:** Only for stable, long-running resources

**3. Spot Instances (ECS Fargate Spot):**
- **Savings:** 50-70% off on-demand
- **Risk:** Can be interrupted (2-minute warning)
- **Use Case:** Non-critical workloads (batch jobs, dev/staging)

**4. Auto-Scaling:**
- **Scale Down:** Reduce TVM servers to 1 during off-hours (e.g., 11 PM - 6 AM)
- **Savings:** 33% reduction in compute cost

**5. S3 Lifecycle Policies:**
```hcl
resource "aws_s3_bucket_lifecycle_configuration" "backups" {
  bucket = aws_s3_bucket.backups.id

  rule {
    id     = "archive-old-backups"
    status = "Enabled"

    transition {
      days          = 90
      storage_class = "GLACIER"  # $0.004/GB/month (vs. $0.023 Standard)
    }

    expiration {
      days = 2555  # 7 years (compliance requirement)
    }
  }
}
```

**6. CloudWatch Logs Retention:**
- **Default:** Indefinite retention
- **Optimized:** 30 days retention (reduce from 90 days)
- **Savings:** 67% reduction in log storage cost

### 14.3 Cost Monitoring

**AWS Cost Explorer:**
- **Weekly Review:** Review cost trends, identify anomalies
- **Budget Alerts:** Set budget ($500/month), alert if exceeded

**Tagging:**
- Tag all resources with: `Environment`, `Project`, `Owner`
- **Cost Allocation:** Track cost per environment (production vs. staging)

```hcl
tags = {
  Environment = "production"
  Project     = "tvm"
  Owner       = "unno-san"
  ManagedBy   = "terraform"
}
```

---

## 15. Security in Deployment

### 15.1 Principle of Least Privilege

**IAM Roles:**
- **TVM Server Role:** Access to RDS, S3 (backups), Secrets Manager (secrets)
- **CI/CD Role (GitHub Actions):** Push to ECR, update ECS services (no EC2 termination, no S3 delete)

**Example IAM Policy (TVM Server):**
```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": ["rds:DescribeDBInstances", "rds:ListTagsForResource"],
      "Resource": "arn:aws:rds:us-east-1:123456789012:db:tvm-production-primary"
    },
    {
      "Effect": "Allow",
      "Action": ["s3:GetObject", "s3:PutObject"],
      "Resource": "arn:aws:s3:::tvm-backups/*"
    },
    {
      "Effect": "Allow",
      "Action": ["secretsmanager:GetSecretValue"],
      "Resource": "arn:aws:secretsmanager:us-east-1:123456789012:secret:tvm/production/*"
    }
  ]
}
```

### 15.2 Secrets in CI/CD

**GitHub Actions Secrets:**
- Store AWS credentials as GitHub Secrets (Settings → Secrets → Actions)
- Use OIDC (OpenID Connect) for GitHub Actions → AWS (no long-lived credentials)

**Example OIDC Configuration:**
```hcl
resource "aws_iam_role" "github_actions" {
  name = "GitHubActionsRole"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Principal = {
          Federated = "arn:aws:iam::123456789012:oidc-provider/token.actions.githubusercontent.com"
        }
        Action = "sts:AssumeRoleWithWebIdentity"
        Condition = {
          StringEquals = {
            "token.actions.githubusercontent.com:sub" = "repo:yourorg/tvm-server:ref:refs/heads/main"
          }
        }
      }
    ]
  })
}
```

**Benefits:** No AWS access keys stored in GitHub (more secure)

### 15.3 Image Scanning

**ECR Image Scanning:**
- **On Push:** Automatically scan Docker images for vulnerabilities (CVEs)
- **Report:** View scan results in ECR console
- **Policy:** Block deployment if CRITICAL vulnerabilities found

**Example: Block Deployment on Critical CVE:**
```bash
# In CI/CD pipeline, after pushing image
SCAN_STATUS=$(aws ecr describe-image-scan-findings --repository-name tvm-server --image-id imageTag=$IMAGE_TAG --query 'imageScanFindings.findingSeverityCounts.CRITICAL')
if [ "$SCAN_STATUS" != "null" ]; then
  echo "CRITICAL vulnerabilities found. Blocking deployment."
  exit 1
fi
```

---

## 16. Disaster Recovery in Deployment

*(See DISASTER_RECOVERY_PLAN.md for full details)*

**Key Deployment Considerations:**

1. **Multi-Region Replication:**
   - **Primary:** us-east-1
   - **Secondary:** us-west-2 (hot standby database, minimal compute)
   - **Failover:** Automated via Route 53 health checks

2. **Immutable Backups:**
   - **S3 Object Lock:** Prevent deletion of backups (ransomware protection)
   - **Cross-Region Replication:** Backups replicated to eu-west-1 (offsite)

3. **Runbooks:**
   - **Disaster Recovery Runbook:** Documented procedures for failover
   - **Stored:** Git repository, printed copies in DR binder

---

## 17. Compliance and Auditing

### 17.1 Audit Logging

**CloudTrail:**
- **Enabled:** All AWS API calls logged to S3
- **Retention:** 7 years (compliance requirement)
- **Monitoring:** GuardDuty analyzes CloudTrail logs for threats

**Application Audit Logs:**
- **What:** All security-relevant events (login, data access, config changes)
- **Where:** Elasticsearch (searchable)
- **Retention:** 1 year (90 days hot, 1 year warm/cold)

### 17.2 Compliance Standards

**SOC 2 Type II:**
- **Requirement:** Annual audit by third-party (if handling sensitive data)
- **Preparation:** Document policies, procedures, controls
- **Evidence:** Audit logs, change logs, access reviews

**ISO 27001:**
- **Requirement:** Information security management system (ISMS)
- **Deployment Aspect:** Infrastructure security controls documented

---

## 18. Deployment Checklist

**Pre-Deployment (Production):**
- [ ] All tests pass (unit, integration, E2E)
- [ ] Security scan passed (no CRITICAL vulnerabilities)
- [ ] Database migrations tested in staging
- [ ] Backup verified (restore tested in last 7 days)
- [ ] Rollback plan documented
- [ ] Change ticket approved (CAB if required)
- [ ] Monitoring dashboards ready
- [ ] On-call engineer paged (aware of deployment)

**During Deployment:**
- [ ] Deploy to staging first
- [ ] Smoke tests in staging pass
- [ ] Deploy to production (rolling/canary)
- [ ] Monitor error rates, latency, CPU, memory
- [ ] Verify health checks pass

**Post-Deployment:**
- [ ] Monitor for 1 hour (watch for anomalies)
- [ ] Verify critical paths (login, create mission, vehicle telemetry)
- [ ] Update status page (if deployment announced)
- [ ] Post-deployment review (what went well, what didn't)

---

## 19. Operational Runbooks

### 19.1 Deploy New TVM Server Version

**Procedure:**
1. Merge PR to `main` branch (triggers CI/CD)
2. GitHub Actions builds Docker image, pushes to ECR
3. Automatic deployment to staging
4. Run integration tests against staging
5. Manual approval for production deployment
6. GitHub Actions updates ECS service (rolling deployment)
7. Monitor production for 1 hour
8. All clear

**Rollback Procedure (If Needed):**
1. Identify previous stable version: `git log --oneline`
2. Trigger redeployment: `aws ecs update-service --cluster tvm-production --service tvm-server --task-definition tvm-server:42` (replace 42 with previous version)
3. Wait 10 minutes for rollback
4. Verify health

### 19.2 Scale TVM Server

**Scale Up (Manual):**
```bash
aws ecs update-service --cluster tvm-production --service tvm-server --desired-count 5
```

**Scale Down:**
```bash
aws ecs update-service --cluster tvm-production --service tvm-server --desired-count 2
```

**Auto-Scaling (Recommended):** Configure target tracking policy (see Section 10.1)

### 19.3 Database Maintenance

**Upgrade PostgreSQL Version:**
1. **Schedule:** Maintenance window (Sunday 4-5 AM UTC)
2. **Backup:** Take manual snapshot before upgrade
3. **Upgrade:** `aws rds modify-db-instance --db-instance-identifier tvm-production-primary --engine-version 15.5 --apply-immediately`
4. **Downtime:** 5-10 minutes (RDS restarts)
5. **Verify:** Check application logs, run smoke tests

---

## 20. Future Enhancements

### 20.1 Multi-Facility Support

**Challenge:** Deploy to multiple facilities (different locations, different networks)

**Solution:**
- **Shared TVM:** Single TVM server (cloud), multiple facilities connect
- **Dedicated TVM:** Each facility has own TVM instance (on-premise or regional cloud)

**Recommendation:** Start with shared TVM, move to dedicated if latency/isolation required

### 20.2 Edge Computing

**Challenge:** Reduce latency for vehicle-TVM communication

**Solution:**
- **AWS Local Zones or Wavelength:** Deploy TVM servers closer to facilities
- **5G Integration:** Ultra-low latency (<10ms) for real-time control

### 20.3 GitOps (ArgoCD)

**Challenge:** Declarative deployment, audit trail of infrastructure changes

**Solution:**
- **ArgoCD:** Automatically sync Kubernetes manifests from Git to cluster
- **Benefits:** Git as source of truth, rollback via `git revert`, change history

---

## Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| **Project Lead** | Maeda-san | __________ | _______ |
| **TVM Lead** | Unno-san | __________ | _______ |
| **DevOps/Infrastructure** | TBD | __________ | _______ |

---

**Document Metadata:**
- **Version:** 1.0
- **Created:** 2025-12-17
- **Next Review:** 2026-03-17 (quarterly review)
- **Owner:** Team 2 (TVM Infrastructure)

---

**Related Documents:**
- `SECURITY_ARCHITECTURE.md` - Network security, IAM roles
- `DISASTER_RECOVERY_PLAN.md` - Backup and failover procedures
- `OBSERVABILITY_ARCHITECTURE.md` - Monitoring and logging details
- `CI_CD_PIPELINE.md` (to be created) - Detailed CI/CD workflows

---

*End of Document*
