# Robotic arm path planner

## Demo
[![Demo video](https://img.youtube.com/vi/v2Hz3039s50/0.jpg)](https://www.youtube.com/watch?v=v2Hz3039s50)

### [Report](outputs/report.md)
### [Path plans](outputs/plans.json)

### Setup 

```bash
cd ~/ros2_ws/src
git clone https://github.com/rajeev-k47/Posha-arm-planner.git posha_robotic_sub
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build
. ~/ros2_ws/install/setup.bash
ros2 launch posha_robotic_sub simulate_assignment.launch.py //rviz2 sim
```
