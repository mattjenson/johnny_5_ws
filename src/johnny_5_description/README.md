# johnny_5_description

Robot description package for the Johnny 5 hobby robot.

## Purpose

This package contains the robot description files that define the physical and visual characteristics of the Johnny 5 robot. In ROS2, the description package serves as the foundation for:

- **Robot Visualization**: Enables viewing the robot model in RViz and other visualization tools
- **Simulation**: Provides the robot model for physics simulators like Gazebo
- **Motion Planning**: Supplies collision geometry and joint limits for path planning algorithms
- **Transform Tree**: Defines the kinematic chain and coordinate frames for sensor fusion and control
- **Hardware Interface**: Describes actuators and sensors for ros2_control integration

## Package Contents

### URDF/Xacro Files (`urdf/`)
- `johnny_5.xacro` - Main robot description file using Xacro macros
- Contains robot links, joints, visual geometry, collision geometry, and physical properties

### Launch Files (`launch/`)
- `view_robot.launch.py` - Display-aware launch file for robot visualization
  - Automatically detects if running with or without display
  - Launches joint state publishers and RViz when appropriate

### Mesh Files (`meshes/`)
- Visual and collision mesh files (STL, DAE, OBJ formats)
- Currently empty - meshes will be added as the robot design evolves

### Configuration
- `package.xml` - Package metadata and dependencies
- `CMakeLists.txt` - Build configuration for mesh installation

## Robot Description Workflow

1. **Xacro Processing**: The `.xacro` files are processed at launch time to generate URDF
2. **Robot State Publisher**: Publishes the robot's kinematic tree to the `/tf` topic
3. **Joint State Publisher**: Provides joint position data (GUI sliders or actual hardware)
4. **Visualization**: RViz displays the robot model using the published transforms

## Key ROS2 Concepts

### URDF (Unified Robot Description Format)
- XML format describing robot kinematics, dynamics, and visual appearance
- Defines links (rigid bodies) and joints (connections between links)

### Xacro (XML Macros)
- Extension of URDF with programmable macros, properties, and includes
- Enables modular, reusable robot descriptions
- Reduces repetition and makes complex robots manageable

### Transform Tree (TF2)
- Publishes coordinate frame relationships between robot parts
- Essential for sensor data fusion and coordinate transformations
- Used by navigation, manipulation, and perception systems

## Usage Examples

### View Robot Model
```bash
# Launch with display (desktop)
ros2 launch johnny_5_description view_robot.launch.py

# Launch headless (SSH/remote)
DISPLAY= ros2 launch johnny_5_description view_robot.launch.py
```

### Use in Other Packages
```xml
<!-- In another package's package.xml -->
<exec_depend>johnny_5_description</exec_depend>
```

```python
# In a launch file
from ament_index_python.packages import get_package_share_directory

robot_description_path = get_package_share_directory('johnny_5_description')
urdf_file = os.path.join(robot_description_path, 'urdf', 'johnny_5.xacro')
```

## Development Status

- ✅ Basic Xacro structure loads in RViz
- ✅ Display-aware launch system
- 🚧 Visual and collision meshes (planned)
- 🚧 Joint limits and dynamics tuning
- ❌ Unit tests for URDF validation

## Related Packages

This description package will be used by:
- `johnny_5_simulation` - Gazebo simulation
- `johnny_5_navigation` - Nav2 stack integration  
- `johnny_5_*_hw` - Hardware interface packages
- `johnny_5_bringup` - System launch files

## Contributing

When adding new robot features:
1. Update the `.xacro` files with new links/joints
2. Add corresponding mesh files to `meshes/`
3. Update `CMakeLists.txt` to install new meshes
4. Test in RViz to verify visualization
5. Run `colcon build` to ensure package builds correctly

## Resources

- [ROS2 URDF Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [robot_state_publisher](https://github.com/ros/robot_state_publisher)
