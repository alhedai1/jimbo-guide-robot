# Guide Robot for Blind People

A ROS2-based autonomous guide robot designed to assist blind and visually impaired individuals with navigation and obstacle avoidance.

## Current Hardware Setup

- **Chassis**: 2-wheel differential drive robot
- **Wheels**: 6.35cm radius, 30cm wheel base
- **Sensors**:
  - RealSense D435i depth camera
  - YDLidar laser scanner
  - UWB sensors (planned)
- **Motor Control**: Arduino-based motor controller via serial communication
- **Processing**: ROS2 Humble on Linux

## Current Software Components

### 1. Motor Interface (`motor_interface_pkg`)
- Serial communication with Arduino motor controller
- Odometry calculation and publishing
- TF transforms for sensor frames
- Velocity command processing

### 2. Safety Monitor (`jimbo_navigation`)
- Multi-sensor obstacle detection (Lidar + Depth camera)
- Emergency stop functionality
- Velocity filtering based on safety status
- Configurable safety zones

### 3. User Follower (`jimbo_navigation`)
- Person detection using lidar clustering
- User tracking and following behavior
- Distance maintenance from user
- Safety-aware navigation

### 4. Sensor Integration
- RealSense camera driver
- YDLidar driver
- TF frame management

## 8-Week Development Roadmap

### Week 1-2: Foundation & Safety Systems
**Priority: Safety and basic functionality**

#### Week 1 Tasks:
- [x] Basic safety monitoring system
- [x] Emergency stop functionality
- [ ] UWB sensor integration
- [ ] Voice feedback system
- [ ] Emergency stop button interface

#### Week 2 Tasks:
- [ ] Enhanced obstacle detection
- [ ] Safety zone visualization
- [ ] Basic user interface testing
- [ ] System integration testing

### Week 3-4: Navigation & Path Planning
**Priority: Core navigation functionality**

#### Week 3 Tasks:
- [ ] SLAM implementation for dynamic mapping
- [ ] Path planning with obstacle avoidance
- [ ] Waypoint following system
- [ ] Navigation stack integration

#### Week 4 Tasks:
- [ ] Enhanced "follow me" behavior
- [ ] Distance maintenance improvements
- [ ] Smooth path following
- [ ] Navigation testing in various environments

### Week 5-6: Advanced Features
**Priority: User experience and reliability**

#### Week 5 Tasks:
- [ ] Multi-sensor fusion implementation
- [ ] Sensor redundancy and confidence scoring
- [ ] Advanced person detection (depth camera)
- [ ] User preference learning

#### Week 6 Tasks:
- [ ] Voice command system
- [ ] Haptic feedback integration
- [ ] User interaction improvements
- [ ] Performance optimization

### Week 7-8: Testing & Refinement
**Priority: Real-world testing and optimization**

#### Week 7 Tasks:
- [ ] Comprehensive indoor testing
- [ ] Outdoor environment testing
- [ ] Different lighting condition testing
- [ ] User scenario testing

#### Week 8 Tasks:
- [ ] Battery life optimization
- [ ] Response time improvements
- [ ] Error handling and recovery
- [ ] Final integration and documentation

## Getting Started

### Prerequisites
- ROS2 Humble
- Python 3.8+
- RealSense SDK
- YDLidar SDK

### Installation
```bash
# Clone the workspace
cd ~/ros2_ws/src

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

### Running the System
```bash
# Launch the complete guide robot system
ros2 launch jimbo_navigation guide_robot.launch.py

# Or launch individual components
ros2 run motor_interface_pkg motor_serial_node
ros2 run jimbo_navigation safety_monitor
ros2 run jimbo_navigation user_follower
```

### Configuration
Key parameters can be adjusted in the launch file:
- `emergency_stop_distance`: 0.3m (30cm)
- `slow_down_distance`: 0.5m (50cm)
- `target_distance`: 1.0m (distance from user)
- `max_linear_velocity`: 0.5 m/s
- `max_angular_velocity`: 1.0 rad/s

## Safety Features

1. **Multi-Sensor Obstacle Detection**: Combines lidar and depth camera data
2. **Emergency Stop**: Immediate halt when obstacles detected within 30cm
3. **Velocity Filtering**: Automatic speed reduction based on proximity
4. **Safety Zones**: Three-tier safety system (warning, slow-down, stop)

## Next Steps

### Immediate Priorities (Week 1-2)
1. **UWB Integration**: Add UWB sensors for precise user tracking
2. **Voice Interface**: Implement basic voice feedback system
3. **Emergency Controls**: Add physical emergency stop button
4. **Testing**: Validate safety systems in controlled environment

### Medium-term Goals (Week 3-6)
1. **Advanced Navigation**: Implement SLAM and path planning
2. **User Experience**: Improve person detection and following
3. **Multi-Sensor Fusion**: Combine all sensor data effectively
4. **Performance**: Optimize for real-time operation

### Long-term Vision (Week 7-8)
1. **Real-world Testing**: Comprehensive testing in various environments
2. **Reliability**: Robust error handling and recovery
3. **Optimization**: Battery life and performance improvements
4. **Documentation**: Complete user and technical documentation

## Contributing

This project is designed for assistive technology applications. Please ensure all changes prioritize safety and reliability.

## License

[Add your license information here]

## Contact

For questions or contributions, please contact [your contact information]. 