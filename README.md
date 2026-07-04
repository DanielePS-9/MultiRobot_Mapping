# Autonomous Multi-Robot Swarm Mapping & Navigation

The aim of the project is to map an unknown simulated environment by deploying a swarm of three TurtleBot3 robots equipped with *Nav2*, *SLAM Toolbox*, and a centralized map merger system.

The entire stack is fully containerized using Docker, featuring a custom ROS 2 Jazzy image.


## Key Features

* **Multi-Robot Coordination:** Supports both *Deterministic* and *Stochastic* swarm exploration behaviors.
* **Centralized Map Merging:** Real-time occupancy grid fusion via `map_merger.py` to generate a unified global map.
* **Dynamic Parameter Management:** Allocation of namespaced frames and configuration fields at runtime to ensure high scalability.


## Repository Structure

```text
.
├── docker_ws/                  # Docker environment setup
│   ├── Dockerfile.prj          # Project container config.
│   ├── build.sh / build_mac.sh # Build scripts 
│   └── entrypoint.sh           # Startup entrypoint
├── ros_ws/                     # ROS 2 Workspace
│   └── src/
│       ├── multirobot_mapping/ # Core swarm logic package 
│       │   ├── config/         # Swarm Nav2 & SLAM config.
│       │   ├── launch/         # Launch files
│       │   └── multirobot_mapping/ # Custom nodes 
│       └── turtlebot3_gazebo/  # Environment models, 
│           │                     meshes, and worlds
│           ├── models/stanza/  # Custom indoor 3D mesh &  
│           │                     SDF config.
│           └── worlds/         # Simulation worlds
├── run.sh / runmac.sh          # Container launchers
├── exec.sh                     # Script to access running 
│                                 container
└── chown_me.sh                 # File permission utility 
                                  script
```
## How to Run the Simulation

Follow these steps to build the containerized environment and launch the multi-robot swarm simulation.

### 1. Build the Docker Image
Navigate to the Docker workspace directory, grant execution permissions to the build script, and compile the image:

```bash
cd docker_ws
chmod +x build.sh
./build.sh
```
### 2. Run the Docker Container
Return to the project root directory and ensure all runtime utility scripts are fully executable, then run the container:
```bash
cd ..
chmod +x run.sh exec.sh chown_me.sh
./run.sh
```
### 3. Launch the simulation 
```bash
colcon build
source install/setup.bash
```
To launch the stochastic swarm exploration:
```bash
ros2 launch multirobot_mapping stochastic_swarm.launch.py
```
To launch the deterministic swarm exploration:
```bash 
ros2 launch multirobot_mapping deterministic_swarm.launch.py
```
### Auxiliary Scripts Reference
`./exec.sh`: Attaches a new interactive shell terminal to the already running Docker container (useful for monitoring topics or debugging nodes mid-simulation).

`./chown_me.sh`: A utility script to fix host-container file permission mismatches on shared workspace volumes.

## Project Report 
A technical report detailing the project's theoretical background, system architecture, algorithm design (deterministic vs. stochastic), and experimental results is available here. 

## Simulation Demo
A video demonstration highlighting the multi-robot system's mapping performance under both exploration algorithms is available here.