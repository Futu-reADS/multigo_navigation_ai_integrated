# Vehicle Software Development Guide

**Team:** Pankaj (Vehicle Software)
**Date:** 2025-12-16
**Version:** 1.0

## ROS 2 Package Structure

```
src/
├── aruco_detect/        # ArUco marker detection
├── nav_control/         # Navigation controller
├── nav_docking/         # Docking controller
├── nav_goal/            # Goal management
├── swerve_drive/        # Swerve drive controller
└── tvm_client/          # TVM fleet client
```

## Building and Testing

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select nav_control

# Run tests
colcon test

# Run specific test
colcon test --packages-select nav_control
```

## Code Standards

- **C++:** Google C++ Style Guide
- **Python:** PEP 8
- **ROS 2:** Follow ROS 2 best practices
- **Testing:** ≥80% code coverage

## Common Development Tasks

### Creating New ROS 2 Node
```bash
cd src/
ros2 pkg create --build-type ament_cmake my_package
```

### Adding Dependencies
Edit `package.xml` and `CMakeLists.txt`, then:
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

---
**References:** All Vehicle requirements and design documents
