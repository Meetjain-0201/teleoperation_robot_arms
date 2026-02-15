# Bilateral Teleoperation System - Dual UR5 Robotic Arms

**Author:** Meet Jain  
**Institution:** Northeastern University, MS in Artificial Intelligence  
**Project Date:** February 2026

## Overview

Real-time bilateral teleoperation system demonstrating task-space control of dual 6-DOF UR5 robotic arms using forward and inverse kinematics. The slave arm follows the master arm's end-effector pose with sub-millimeter accuracy (<1mm) at 100Hz control frequency.

## Technical Architecture

### System Components

1. **Master Arm**: Operator-controlled UR5 arm (manual input via GUI sliders)
2. **Slave Arm**: Automatically follows master using analytical IK
3. **Control Loop**: 100Hz real-time task-space synchronization

### Control Flow
```
Master Joints → Forward Kinematics → End-Effector Pose (x,y,z,R) 
                                            ↓
                                    Inverse Kinematics
                                            ↓
                                      Slave Joints
```

### Key Features

- **Forward Kinematics**: DH parameter-based transformation matrices
- **Inverse Kinematics**: Analytical closed-form solution (8 possible configurations, selects optimal)
- **Tracking Accuracy**: Position error < 1mm consistently
- **Control Frequency**: 100Hz update rate
- **Robot Model**: UR5 with realistic URDF meshes

## Dependencies

- ROS2 Humble
- Python 3.10+
- ur-analytic-ik (`pip3 install ur-analytic-ik`)
- ur_description package (UR5 URDF models)

## Installation
```bash
# Clone repository
git clone git@github.com:Meetjain-0201/teleoperation_robot_arms.git
cd teleoperation_robot_arms

# Install dependencies
rosdep install --from-paths src -y --ignore-src
pip3 install ur-analytic-ik

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Demo
```bash
ros2 launch teleop_gazebo teleop_rviz_demo.launch.py
```

This launches:
- Robot state publisher
- Joint state publisher GUI (master control)
- Teleoperation node (FK→IK computation)
- RViz visualization

### Control Master Arm

**Method 1: GUI Sliders**
- Use joint_state_publisher_gui window
- Move sliders to control master joints
- Slave automatically follows in real-time

**Method 2: Command Line**
```bash
ros2 topic pub --once /master_position_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.5, -1.0, 1.0, -0.5, -1.57, 0.0]"
```

### View in RViz

1. Set Fixed Frame to `world`
2. Add display: RobotModel
3. Add display: TF (optional - shows coordinate frames)
4. Observe both UR5 arms - slave mirrors master's end-effector pose

## Technical Implementation

### Forward Kinematics
- Uses standard DH parameters for UR5
- Computes 4x4 homogeneous transformation matrix
- Transforms joint angles → end-effector pose

### Inverse Kinematics
- Analytical solution (not numerical optimization)
- Computes all 8 possible joint configurations
- Selects solution closest to current configuration (smooth motion)
- ~10 microseconds per solve (fast enough for real-time)

### Package Structure
```
teleoperation_ws/
├── src/
│   ├── teleop_description/       # Robot URDF models
│   │   ├── urdf/
│   │   │   ├── complete_system.urdf.xacro
│   │   │   └── combined.ros2_control.xacro
│   │   └── config/
│   ├── teleop_controllers/       # Kinematics & control nodes
│   │   ├── ur5_analytical_ik.py
│   │   ├── teleoperation_node.py
│   │   ├── master_control_bridge.py
│   │   └── kinematics.py
│   └── teleop_gazebo/           # Launch files
│       └── launch/
│           └── teleop_rviz_demo.launch.py
```

## Performance Metrics

- **Control Loop Frequency**: 100Hz
- **Position Tracking Error**: <1mm (typically 0.00-0.50mm)
- **IK Solve Time**: ~10 microseconds per iteration
- **Joint Synchronization**: Perfect (slave mirrors master pose exactly)

## Applications

This system demonstrates core concepts used in:
- Surgical robotics (da Vinci-style teleoperation)
- Remote manipulation systems
- Master-slave robotic control
- Real-time inverse kinematics

## Related Experience

This project builds on experience from:
- **Articulus Surgical**: Developed IK for 7-DOF surgical manipulator with ~1cm accuracy
- **Mars Rover Manipal**: Autonomous navigation with sensor fusion
- **ARTPARK IISc**: SLAM-based localization and path planning

## License

Apache License 2.0

## Contact

**Meet Jain**  
MS in Artificial Intelligence, Northeastern University  
Email: jain.meet@northeastern.edu  
GitHub: [Meetjain-0201](https://github.com/Meetjain-0201)
