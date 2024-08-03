# ROS2 DRL Robot Navigation Project

This document provides instructions to set up and run the ROS2-based DRL Robot Navigation project.

## Project Structure

```
text
TD3
├── __pycache__
│   ├── replay_buffer.cpython-38.pyc
│   ├── replay_buffer2.cpython-38.pyc
│   └── velodyne_env.cpython-38.pyc
├── assets
│   ├── multi_robot_scenario.launch
│   └── multi_robot_scenario.launch.py
├── pytorch_models
│   ├── TD3_velodyne_actor.pth
│   ├── TD3_velodyne_critic.pth
│   └── description
├── results
│   ├── TD3_velodyne.npy
│   └── description
├── runs
│   ├── Jul10_10-26-18_vtx
│   ├── Jul10_10-28-12_vtx
|   |...
│   ├── Jul11_09-52-09_vtx
│   ├── Jul11_09-57-52_vtx
│   ├── May24_11-27-38_vtx
│   └── description
├── small_house
│   ├── maps
│   ├── models
│   ├── photos
│   └── small_house.world
└── src
    ├── CMakeLists.txt
    ├── package.xml
    ├── replay_buffer.py
    ├── test_velodyne_td3.py
    ├── train_velodyne_td3.py
    └── velodyne_env.py
```

## Setup Instructions

### Step 1: Clone the Project Repository

```bash
git clone <repository-url>
cd TD3
```

### Step 2: Clone TurtleBot3 and Dependencies

Navigate to your `src` folder:

```bash
cd src
```

Clone the TurtleBot3 repositories:

```bash
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

### Step 3: Install Dependencies

Make sure you have all the necessary dependencies installed:

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Build Your Workspace

Navigate back to the root of your workspace and build it using colcon:

```bash
cd ..
colcon build
```

### Step 5: Source the Setup Script

Source the setup script to overlay this workspace on top of your environment:

```bash
source install/setup.bash
```

## Running the Project

### Step 1: Launch the Simulation

In one terminal, navigate to the root of your workspace and launch the simulation environment:

```bash
source install/setup.bash
ros2 launch <your-package-name> multi_robot_scenario.launch.py
```

### Step 2: Run the DRL Test Script

In another terminal, navigate to the root of your workspace, source the setup script, and run the test script:

```bash
source install/setup.bash
python3 src/test_velodyne_td3.py
```

This will start the DRL testing process using the TurtleBot3 in the Gazebo simulation environment.
