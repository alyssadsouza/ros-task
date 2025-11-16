# CPP with LIMO Bots

## Requirements

### Linux VM
We need to run the simulation software on Linux because of dependencies that aren't available on Mac so we run this on a Linux VM. We're using an EC2 instance to run the simulation setup because the Nav2 stack requires more RAM than our Macs running UTM can give. See [EC2 // NoMachine setup](https://www.notion.so/EC2-NoMachine-setup-2a50ab55dac6807eba17f999e6a57f5f?pvs=21). 

*However*, there can only be one NoMachine instance connected to the VM at once, so:
- For local development we use [UTM](https://getutm.app/) running Ubuntu 22, it'll just be slow when running rviz and certain compute intensive features like the path smoothing node won't be able to run
- When you want to perform e2e visualizations + testing use EC2 w/ NoMachine (make sure you've pulled the changes from your local repo)

### Clone repository
```bash
git clone https://github.com/alyssadsouza/ros-task
```
> 💡 This repository should already be cloned on the EC2 instance.

## Setup simulator

### Build docker container
```bash
./scripts/build/sim.sh --no-cache
```

### Run container
```bash
# Running the container
./scripts/deploy/devel.sh -c # -c for CPU mode instead of GPU
```

If you want to enter another instance of the container while it's already running:
```bash
docker exec -it limo_bot bash
```

### Run simulation
> ⚠️ Make sure you're in the docker container first.

1. Add environment variables to enable Mesa software rendering:
    
    ```bash
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_GL_VERSION_OVERRIDE=3.3
    export GALLIUM_DRIVER=llvmpipe
    ```
    
2. Build ROS nodes:
    
    ```bash
    source /opt/ros/humble/setup.bash
    
    cd /root/workspace
    
    # Re-run the following on every change:
    colcon build
    source install/setup.bash
    ```
    
3. Launch simulation:
    
    ```bash
    ros2 launch limo_simulation limo.launch.py
    ```
    
4. Launch Nav2 (in another terminal):
    
    ```bash
    ros2 launch limobot_nav2_config nav2_limo.launch.py
    ```
    
5. Launch RViz (in another terminal):
    
    ```bash
    rviz2
    ```
    
### Using teleop
```bash
# Install package
apt-get install ros-humble-teleop-twist-keyboard
# Run teleop node
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Working with DARP

Clone the repo (outside of the docker container):
```bash
cd workspace
git clone https://github.com/alice-st/DARP darp_python
```

Now rebuild the docker container with `/scripts/deploy/devel.sh` and run the commands below.

### Build and run the DARP ROS node:
```bash
cd /root/workspace
# Build the ROS node
colcon build --packages-select darp_ros2 && source install/setup.bash
# Run the node
ros2 run darp_ros2 simple_planner
```

### Unit testing:
```bash
ros2 run darp_ros2 coordinate_converter_test
```

### Run DARP with pygame visualization:
```bash
ros2 run darp_ros2 simple_planner --ros-args -p visualize_darp:=true
```

## Full End-to-End Coverage Path Planning with Nav2

This workflow runs the complete system: simulator → Nav2 navigation stack → DARP coverage planner → path execution.

### Setup (Terminal 1)

Start the simulator:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe

source /opt/ros/humble/setup.bash
cd /root/workspace
colcon build
source install/setup.bash

ros2 launch limo_simulation limo.launch.py
```

### Nav2 Stack (Terminal 2)

Launch the navigation stack:
```bash
source /opt/ros/humble/setup.bash
source /root/workspace/install/setup.bash

ros2 launch limobot_nav2_config nav2_limo.launch.py
```

### Visualization (Terminal 3)

Launch RViz:
```bash
rviz2
```

Add these displays to visualize the coverage path planning:
- **Marker Array** - Topic: `/darp_path_markers` (green→red waypoint spheres, red path line)
- **Path** - Topic: `/darp_coverage_path` (green line showing planned path)
- **Path** - Topic: `/received_global_plan` (yellow line showing Nav2's execution)
- **Robot Model** (should appear automatically)

### Coverage Planning (Terminal 4)

Run the DARP coverage planner node:
```bash
source /opt/ros/humble/setup.bash
source /root/workspace/install/setup.bash

ros2 run darp_ros2 simple_planner
```

The node will:
1. ✓ Run DARP algorithm (2.42s for 10×10 grid)
2. ✓ Generate 401 waypoint coverage path
3. ✓ Publish path to RViz for visualization
4. ✓ Send path to Nav2 follow_path action
5. ✓ Robot executes the coverage pattern

### Monitoring execution

You can monitor the execution with:
```bash
# Check DARP planner logs
ros2 topic echo /darp_coverage_path

# Monitor controller output
ros2 topic echo /cmd_vel

# Check robot odometry
ros2 topic echo /odom
```

## Development workflow

For faster iteration during development:

```bash
# Terminal 1: Simulator
ros2 launch limo_simulation limo.launch.py

# Terminal 2: Nav2
ros2 launch limobot_nav2_config nav2_limo.launch.py

# Terminal 3: RViz
rviz2

# Terminal 4: Edit and rebuild DARP node
cd /root/workspace
colcon build --packages-select darp_ros2
source install/setup.bash
ros2 run darp_ros2 simple_planner
```

Every time you make changes to `simple_planner_node.py`, rebuild with `colcon build --packages-select darp_ros2` and re-run the node.
