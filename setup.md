# Setup

## 1. Clone this repo


## 2. Download world assets
Download robot_worlds.tar.gz from: https://drive.google.com/file/d/1wuR_AM7Db0KZtmQFM8753OmAgnauNj7z/view?usp=sharing
Extract directly into your home folder:

    mkdir -p ~/robot_worlds

This creates ~/robot_worlds/worlds/small_house/ — matching the
original project structure exactly.

## 3. Add to ~/.bashrc
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/robot_worlds/worlds/small_house/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/robot_worlds/worlds/small_house
source ~/.bashrc

## 4. Create your own .env
Create ~/nav_ws/.env with:
GEMINI_API_KEY=your_own_key_here

## 5. Build
cd ~/nav_ws
colcon build
source install/setup.bash

## 6. Launch
- ros2 launch nav_robot perception.launch.py
- ros2 launch nav_robot brain.launch.py
- ros2 launch nav_robot robot.launch.py
- ros2 run nav_robot nav2_goal_node