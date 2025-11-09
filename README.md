# Task Description

Please read through [this](https://www.overleaf.com/read/dmgrrcmpkbkq#211e69) document before moving forward.

### Software Structure
```
- docker -- Where the Dockerfile lives.
- scripts -- Where necessary external scripts live.
- workspace -- Where all the packages live.
```

### Build the simulator

```bash
./scripts/build/sim.sh
```

### Run the simulator

```bash
./scripts/deploy/devel.sh # To enter the docker container
ros2 launch limo_simulation limo.launch.py # To launch the simulator
```

### What do I edit?

1. Modify the package `limo_control` in the workspace directory for adding your c++ controller program.
2. Make a launch file that can launch everything (Controller and Simualation).
3. Modify `scripts/deploy/app.sh` such that, when `scripts/deploy/start.sh` is run, the task is executed automatically.

### Known Issues

1. This will not work with docker desktop, please do not use it, use the default engine.

Feel free to modify anything else if it does not work as expected.

### Working with DARP

1. colcon build --packages-select darp_ros2 && source install/setup.bash
2. ros2 run darp_ros2 simple_planner --ros-args -p visualize_darp:=true
3. ros2 run darp_ros2 coordinate_converter_test

In terminal 1
```
cd /home/ubuntu/Documents/ros-task/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run darp_ros2 simple_planner
```

In terminal 2
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /coverage_path
ros2 topic echo /coverage_path --once
```

In terminal 3
```
rviz2
```

- Click "Add" button
- Select "By topic" tab
- Find /coverage_path → Path

Step 1: Launch Development Container + Gazebo Simulation
Terminal 1:
cd /home/ubuntu/Documents/ros-task
./scripts/deploy/devel.sh
Once inside the container:
cd /root/workspace
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch limo_simulation limo.launch.py
Wait: ~15-20 seconds for Gazebo to fully load.
Step 2: Launch Nav2 Smoother Server
Terminal 2:
docker exec -it limo_bot bash
Inside container:
cd /root/workspace
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch darp_ros2 darp_smoother.launch.py
Expected output:
[smoother_server]: Created smoother : ConstrainedSmoother of type nav2_constrained_smoother/ConstrainedSmoother
[smoother_lifecycle_manager]: Managed nodes are active
[path_smoother_client]: Path smoother client ready: /coverage_path -> /smoother_server/smooth_path -> /coverage_path_smooth
Step 3: Run DARP Planner
Terminal 3:
docker exec -it limo_bot bash
Inside container:
cd /root/workspace
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run darp_ros2 simple_planner
Expected output:
[simple_darp_planner]: ✓ DARP completed successfully
[simple_darp_planner]: Publishing EXTREME downsampled path: 5 poses (from original 401)
[simple_darp_planner]: ✓ Published coverage path: 401 waypoints, ~200.5m total length
Then in Terminal 2, you should see:
[path_smoother_client]: Submitted path with 5 poses to smoother
[path_smoother_client]: Published smoothed path with X poses  # ← SUCCESS (if it works)
Step 4: Visualize in RViz2
Terminal 4 (on your Mac host, not in Docker): If you have ROS 2 Humble installed on macOS:
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0  # Match container's domain
rviz2
In RViz2:
Set Fixed Frame: Click "Fixed Frame" dropdown → Select map
Add Raw Path:
Click "Add" button (bottom left)
By topic → /coverage_path → Path
Set color to Red
Add Smoothed Path:
Click "Add" again
By topic → /coverage_path_smooth → Path
Set color to Green
Compare: You should see red (original downsampled) vs green (smoothed)
Alternative: Monitor Topics (Instead of RViz2)
Terminal 4:
docker exec -it limo_bot bash
cd /root/workspace
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Check if smoothed path is publishing
ros2 topic hz /coverage_path_smooth

# Or echo to see the actual path
ros2 topic echo /coverage_path_smooth --once