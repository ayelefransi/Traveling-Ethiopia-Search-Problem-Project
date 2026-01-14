# Traveling Ethiopia - ROS 2 Robot Navigation Project

A complete ROS 2 simulation implementing the Traveling Ethiopia search problem with autonomous robot navigation through Ethiopian cities using Breadth-First Search (BFS) algorithm.

## Project Overview

This project demonstrates an intelligent robot navigation system that:
- Uses an uninformed search strategy (BFS) to find optimal paths between Ethiopian cities
- Autonomously navigates a three-wheeled differential drive robot through waypoints
- Simulates realistic Ethiopian terrain and landmarks in Gazebo
- Implements sensor integration (Lidar, IMU, RGB Camera)

## Features

### Robot Capabilities
- **Three-wheeled differential drive robot** with realistic physics
- **Lidar sensor** - 360-degree laser scanner (10m range)
- **IMU sensor** - Gyroscope and accelerometer (100Hz)
- **RGB Camera** - 640x480 resolution at 30fps
- **Autonomous navigation** using proportional control

### World Environment
- **45 Ethiopian cities** represented as location pin markers
- **Color-coded regions** for easy identification
- **Ethiopian landmarks** including Simien Mountains, Lake Tana, Bale Mountains, and Rift Valley Lakes
- **Realistic terrain** with Ethiopian Highlands coloring
- **Interactive markers** - Hover over pins in Gazebo to see city names

### Path Planning
- **Breadth-First Search (BFS)** algorithm for finding shortest paths
- **Graph-based navigation** with 45 nodes and their connections
- **Real-time path visualization** and logging
- **Waypoint-following** navigation system

## System Requirements

- **OS**: Ubuntu 24.04 (or WSL2 with Ubuntu 24.04)
- **ROS 2**: Jazzy Jalisco
- **Gazebo**: Harmonic (gz-sim)
- **Python**: 3.12+

## Installation

### 1. Install ROS 2 Jazzy
```bash
# Follow official ROS 2 Jazzy installation guide
# https://docs.ros.org/en/jazzy/Installation.html
```

### 2. Install Gazebo Harmonic
```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

### 3. Clone and Build
```bash
cd ~/your_workspace
git clone <repository-url>
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Quick Start (Recommended)

Launch everything with one command:

```bash
ros2 launch traveling_ethiopia complete_demo.launch.py
```

This will:
1. Start Gazebo with the Ethiopian world
2. Spawn the robot at Addis Ababa
3. Calculate path from Addis Ababa to Harar using BFS
4. Automatically navigate the robot through the path

### Custom Path Planning

Specify different start and goal cities:

```bash
ros2 launch traveling_ethiopia complete_demo.launch.py start_city:=Gambela goal_city:=Jijiga
```

### Manual Launch (Step-by-Step)

**Terminal 1 - Launch Simulation:**
```bash
ros2 launch traveling_ethiopia gazebo_main.launch.py
```

**Terminal 2 - Run Path Planner:**
```bash
source install/setup.bash
ros2 run traveling_ethiopia path_planner --ros-args -p start_city:=Addis_Ababa -p goal_city:=Harar
```

**Terminal 3 - Start Robot Navigation:**
```bash
source install/setup.bash
ros2 run traveling_ethiopia robot_driver
```

## Available Cities

The simulation includes 45 Ethiopian cities organized by region:

**Western Ethiopia (Red pins):**
Gambela, Dembi Dollo, Gore, Gimbi, Tepi, Mezan Teferi

**Central-West (Green pins):**
Nekemte, Bedelle, Jimma, Bonga, Dawro, Wolait Sodo, Hossana

**Central (Blue pins):**
Ambo, Wolkite, Worabe, Hawassa, Dilla

**Capital Region (Yellow/Gold pins):**
Addis Ababa, Debra Birhan, Buta Jirra, Adama, Batu, Shashemene

**Central-East (Magenta pins):**
Matahara, Assella, Assasa, Dodola, Bale

**Eastern Ethiopia (Cyan pins):**
Awash, Chiro, Dire Dawa, Harar, Babile, Jijiga

**Far Regions (Gray pins):**
Dega Habur, Goba, Sof Oumer, Kebri Dehar, Gode

## Example Paths

**Short Path:**
```bash
ros2 launch traveling_ethiopia complete_demo.launch.py start_city:=Addis_Ababa goal_city:=Harar
```
Expected path: Addis_Ababa → Adama → Matahara → Awash → Chiro → Dire_Dawa → Harar

**Long Path:**
```bash
ros2 launch traveling_ethiopia complete_demo.launch.py start_city:=Gambela goal_city:=Jijiga
```
Tests BFS across the entire graph

**Central Route:**
```bash
ros2 launch traveling_ethiopia complete_demo.launch.py start_city:=Jimma goal_city:=Assella
```
Navigation through central cities

## Monitoring and Debugging

### View Sensor Data

**Lidar:**
```bash
ros2 topic echo /lidar
```

**IMU:**
```bash
ros2 topic echo /imu
```

**Camera:**
```bash
ros2 run rqt_image_view rqt_image_view /camera
```

### Check Active Topics
```bash
ros2 topic list
```

### Monitor Robot Odometry
```bash
ros2 topic echo /odom
```

### View Path Planning Output
```bash
ros2 topic echo /planned_path
```

## Project Structure

```
traveling_ethiopia/
├── launch/
│   ├── gazebo_main.launch.py          # Main simulation launcher
│   └── complete_demo.launch.py        # All-in-one demo launcher
├── traveling_ethiopia/
│   ├── graph_data.py                  # City graph structure
│   ├── path_planner.py                # BFS path planning node
│   └── robot_driver.py                # Waypoint navigation controller
├── urdf/
│   └── robot.urdf.xacro               # Robot description with sensors
├── worlds/
│   └── ethiopia.world                 # Gazebo world with cities and landmarks
├── scripts/
│   └── generate_pin_world.py          # World generation script
├── package.xml                         # ROS 2 package manifest
└── setup.py                           # Python package setup
```

## Technical Details

### Search Algorithm
- **Algorithm**: Breadth-First Search (BFS)
- **Graph**: 45 nodes (cities) with bidirectional edges
- **Optimality**: Finds shortest path in terms of number of cities
- **Completeness**: Guaranteed to find a solution if one exists

### Robot Control
- **Type**: Differential drive with caster wheel
- **Control**: Proportional controller for steering
- **Position tolerance**: 0.3 meters
- **Angle tolerance**: 0.1 radians
- **Linear speed**: 0.5 m/s
- **Angular speed**: 1.0 rad/s

### Sensors
- **Lidar**: 360 samples, 10m range, 10Hz update rate
- **IMU**: 100Hz update rate with Gaussian noise
- **Camera**: 640x480 resolution, 30Hz, 60-degree FOV

## Troubleshooting

### Robot not visible in Gazebo
```bash
# Rebuild the package
colcon build --symlink-install
source install/setup.bash
```

### Path planner not finding path
- Ensure city names use underscores (e.g., `Addis_Ababa` not `Addis Ababa`)
- Check available cities in `graph_data.py`

### Robot not moving
- Verify all three nodes are running (gazebo, path_planner, robot_driver)
- Check topic connections: `ros2 topic list`
- Ensure `/cmd_vel` and `/odom` topics are active

### Gazebo crashes on launch
- Check Gazebo Harmonic installation
- Verify world file syntax: `gz sdf -k worlds/ethiopia.world`

## Academic Context

This project implements the "Traveling Ethiopia" search problem as an interactive intelligent system assignment, demonstrating:

1. **Uninformed search strategy** (BFS) for pathfinding
2. **Robot design** with functional physics and sensors
3. **World modeling** using Cartesian coordinate system
4. **Autonomous navigation** from initial to goal states

## Contributing

This is an academic project. For improvements or bug fixes, please follow standard ROS 2 coding conventions and ensure all changes are tested in simulation.

## License

This project is created for educational purposes.

## Authors

Developed as part of an Interactive Intelligent Systems course assignment.

## Acknowledgments

- ROS 2 Jazzy community
- Gazebo Harmonic simulation environment
- Ethiopian geography and cultural heritage
