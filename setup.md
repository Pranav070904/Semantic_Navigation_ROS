# Semantic Navigation ROS Setup & Architecture Guide

## Main Setup

## 1. Clone this repo


## 2. Download world assets
Download robot_worlds.tar.gz from: https://drive.google.com/file/d/1wuR_AM7Db0KZtmQFM8753OmAgnauNj7z/view?usp=sharing
Extract directly into your home folder:
```
    mkdir -p ~/robot_worlds
```

This creates ~/robot_worlds/worlds/small_house/ — matching the
original project structure exactly.

## 3. Add to ~/.bashrc
```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/robot_worlds/worlds/small_house/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/robot_worlds/worlds/small_house
source ~/.bashrc
```
## 4. Create your own .env
```
Create ~/nav_ws/.env with:
```
GEMINI_API_KEY=your_own_key_here

## 5. Build
```
cd ~/nav_ws
colcon build
source install/setup.bash
```

## 6. Launch
```
- ros2 launch nav_robot perception.launch.py
- ros2 launch nav_robot brain.launch.py
- ros2 launch nav_robot robot.launch.py
- ros2 run nav_robot nav2_goal_node
```

This document provides complete instructions for setting up, building, running, and testing the **Semantic Navigation ROS** project.

---
## Modified

## 📋 1. Project Directory Structure

```
Semantic_Navigation_ROS-main/
├── .env                              # Environment configuration (GEMINI_API_KEY)
├── setup.md                          # Project setup & architecture guide
└── src/
    └── nav_robot/
        ├── config/
        │   ├── nav2_params.yaml      # Nav2 navigation costmap & planner params
        │   ├── rooms.yaml            # Coordinate map for indoor rooms
        │   └── slam_toolbox.yaml     # SLAM configuration
        ├── launch/
        │   ├── brain.launch.py       # Launches Orchestrator & LLM Reasoning Node
        │   ├── perception.launch.py  # Launches Grounding DINO & Local Search
        │   └── robot.launch.py       # Launches Gazebo Sim, Robot Publisher & Nav2
        ├── nav_robot/
        │   ├── grounding_dino_node.py # Zero-Shot perception node (Transformers)
        │   ├── local_search_node.py   # Visual servoing & search pattern controller
        │   ├── nav2_goal_node.py      # Nav2 action client for room navigation
        │   ├── orch_node.py           # High-level state machine orchestrator
        │   └── reasoning_node.py      # Gemini 3.6 Flash reasoning & misclassifications
        ├── package.xml
        └── setup.py
```

---

## 🛠 2. Prerequisites & World Assets

### A. Download & Extract World Models
Download `robot_worlds.tar.gz` and extract into your home folder:
```bash
mkdir -p ~/robot_worlds
# Extract contents to match: ~/robot_worlds/worlds/small_house/
```

### B. Environment Variables (`~/.bashrc`)
Add the Gazebo model resource paths to your `~/.bashrc`:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/robot_worlds/worlds/small_house/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/robot_worlds/worlds/small_house
source ~/.bashrc
```

---

## 🐍 3. Python AI/ML Dependencies

Install required libraries for zero-shot object detection and Gemini LLM reasoning:
```bash
pip install --break-system-packages torch transformers google-genai python-dotenv
```

---

## 🔑 4. API Key Configuration

Create `.env` inside the workspace root folder:
```bash
GEMINI_API_KEY=your_actual_gemini_api_key_here
```

---

## 🔧 5. System Portability & Code Modifications Made

The codebase was updated to eliminate hardcoded paths (`/home/...`) making it 100% portable across any system:

1. **[reasoning_node.py](file:///home/student/23brs1358/Semantic_Navigation_ROS-main/src/nav_robot/nav_robot/reasoning_node.py)**:
   - System-agnostic `.env` resolution using `find_dotenv(usecwd=True)` + `os.path.expanduser('~/.env')`.
   - Added ROS 2 publisher for **`/possible_misclassifications`** (`std_msgs/msg/String`).
   - Configured Gemini prompt to generate **at least 10 potential visual misclassifications** for surrounding indoor objects (e.g. fridge misclassified as door, sofa as bed, microwave as oven).

2. **[nav2_goal_node.py](file:///home/student/23brs1358/Semantic_Navigation_ROS-main/src/nav_robot/nav_robot/nav2_goal_node.py)**:
   - System-agnostic package lookup `get_package_share_directory('nav_robot')` for loading `config/rooms.yaml`, with a relative source script fallback (`os.path.dirname(__file__)`).

3. **[robot.launch.py](file:///home/student/23brs1358/Semantic_Navigation_ROS-main/src/nav_robot/launch/robot.launch.py)**:
   - Portable Gazebo `world_file` path via `os.path.expanduser('~/robot_worlds/worlds/small_house/small_house.world')`.
   - Dynamic `yaml_filename` parameter override for `map_server_node` so `nav2_params.yaml` loads map files dynamically on any system without manual edits.

---

## 🏗 6. Build the ROS 2 Package

```bash
cd Semantic_Navigation_ROS-main
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## 🚀 7. Execution & Launch Instructions

Open 4 separate terminal tabs:

### Terminal 1: Perception Stack
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nav_robot perception.launch.py
```

### Terminal 2: Brain & Reasoning Stack
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nav_robot brain.launch.py
```

### Terminal 3: Robot Simulation Stack
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/robot_worlds/worlds/small_house/models:~/robot_worlds/worlds/small_house
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nav_robot robot.launch.py
```

### Terminal 4: Nav2 Goal Node
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run nav_robot nav2_goal_node
```

---

## 🧪 8. Testing & Verification

Publish a natural language target goal to the `/goal` topic:
```bash
ros2 topic pub --once /goal std_msgs/msg/String "{data: 'fridge'}"
```

### Expected Output & Topic Monitoring
* Listen to visual misclassification warnings (at least 10 items):
  ```bash
  ros2 topic echo /possible_misclassifications
  ```
* View high-level reasoning actions:
  ```bash
  ros2 topic echo /reasoning_action
  ```

