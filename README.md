# MiniM5Robo
MiniM5Robo firm

## Host Setting
```bash
source /opt/ros/humble/setup.bash

mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep install --from-paths src --ignore-src -y
colcon build

source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

## run agent
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```
