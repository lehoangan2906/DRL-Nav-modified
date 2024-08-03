# TD3 Robot Navigation Project

This guide provides instructions on how to build and run the TD3 Robot Navigation project using ROS2.

## Project Structure

```
text

TD3
├── __pycache__
│   ├── replay_buffer.cpython-38.pyc
│   ├── replay_buffer2.cpython-38.pyc
│   └── velodyne_env.cpython-38.pyc
├── assets
│   ├── multi_robot_scenario.launch
│   └── multi_robot_scenario.launch.py
├── pytorch_models
│   ├── TD3_velodyne_actor.pth
│   ├── TD3_velodyne_critic.pth
│   └── description
├── results
│   ├── TD3_velodyne.npy
│   └── description
├── runs
│   ├── Jul10_10-26-18_vtx
│   |...
│   ├── May24_11-27-38_vtx
│   └── description
├── small_house
│   ├── maps
│   ├── models
│   ├── photos
│   └── small_house.world
└── src
    ├── CMakeLists.txt
    ├── package.xml
    ├── replay_buffer.py
    ├── test_velodyne_td3.py
    ├── train_velodyne_td3.py
    └── velodyne_env.py
```

## Prerequisites

- ROS2 Foxy (or compatible version)
- colcon (build tool for ROS2)
- Gazebo (compatible version with ROS2 Foxy)
- Python3
- PyTorch

## Setup Instructions

### 1. Clone the Project Repository

```bash
git clone <repository_url>
cd TD3
```

### 2. Build the Workspace

Navigate to the root of your workspace (where the `src` directory is located) and build the workspace using `colcon`:

```bash
colcon build
```

### 3. Source the Setup Script

After the build completes, source the setup script to overlay this workspace on your environment:

```bash
source install/setup.bash
```

### 4. Run the Launch File

To launch the Gazebo environment, use the following command:

```bash
ros2 launch TD3 multi_robot_scenario.launch.py
```

### 5. Run the Test Script

To run the test script for the deep reinforcement learning model, use the following command:

```bash
python3 src/test_velodyne_td3.py
```

### Additional Notes

- Make sure to source the appropriate setup files to include your ROS2 environment variables:

  ```bash
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  ```

- Verify that your ROS2 and Gazebo versions are compatible.

