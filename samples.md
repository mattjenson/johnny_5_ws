# Johnny-5 Samples

## Base

### Build & Launch
From the workspace root (first terminal):

```bash
# Build just the base driver + description packages
colcon build --packages-select johnny_5_description johnny_5_base_driver
# Source the workspace
source install/setup.bash
# (Optional) visualize robot (robot_state_publisher + joint states + RViz if display)
ros2 launch johnny_5_description view_robot.launch.py
# Bring up base (robot_state_publisher + base_driver)
ros2 launch johnny_5_base_driver base_bringup.launch.py
```

Alternative: run only the driver (assumes robot description already running somewhere):
```bash
ros2 run johnny_5_base_driver base_driver
```

New terminal sessions must re-source:
```bash
cd /home/matt/johnny_ws
source install/setup.bash
```

Prerequisites (each terminal session):
1. From workspace root:
   source install/setup.bash
2. Make sure the base driver (and robot description if desired) is running:
   ros2 launch johnny_5_base_driver base_bringup.launch.py

Topic used: /cmd_vel (geometry_msgs/msg/Twist)

### Single forward movement (publish once)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

### Continuous forward (repeats at default rate until Ctrl-C)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

### Rotate in place (left track backward, right track forward)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

### Forward while turning left
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.3}}"

### Stop (zero everything)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

### Inspect incoming velocity commands
ros2 topic echo /cmd_vel

### Inspect odometry output
ros2 topic echo /odom

Notes:
- linear.x is forward speed in m/s.
- angular.z is yaw rate in rad/s (positive = counter‑clockwise).
- Use -1 with ros2 topic pub to send a single message then exit.
- Combine linear.x and angular.z for arcs.
