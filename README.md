# CPP with LIMO Bots

## Requirements

### Linux VM

We need to run the simulation software on Linux because of dependencies that aren't available on Mac so we run this on a Linux VM. We’re using an EC2 instance to run the simulation setup because the Nav2 stack requires more RAM than our Macs running UTM can give. See [EC2 // NoMachine setup](https://www.notion.so/EC2-NoMachine-setup-2a50ab55dac6807eba17f999e6a57f5f?pvs=21). 

*However*, there can only be one NoMachine instance connected to the VM at once, so:

- For local development we use [UTM](https://getutm.app/) running Ubuntu 22, it’ll just be slow when running rviz and certain compute intensive features like the path smoothing node won’t be able to run
- When you want to perform e2e visualizations + testing use EC2 w/ NoMachine (make sure you’ve pulled the changes from your local repo)

### Clone repository

```bash
git clone https://github.com/alyssadsouza/ros-task
```

> 💡 This repository should already be cloned on the EC2 instance. We’re currently working on the `darp_setup` branch.

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

If you want to enter another instance of the container while it’s already running:

```bash
docker exec -it limo_bot bash
```

### Run simulation

> ⚠️ Make sure you’re in the docker container first.

1. Add environment variables to enable Mesa software rendering:
    
    ```bash
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_GL_VERSION_OVERRIDE=3.3
    export GALLIUM_DRIVER=llvmpipe
    ```
    
2. Build ROS node:
    
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
    
4. Launch RVIZ (do this in another terminal of the container):
    
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

### Working with DARP

Clone the repo (outside of the docker container):

```bash
cd workspace
git clone https://github.com/alice-st/DARP darp_python
```

Now rebuild the docker container with `/scripts/deploy/devel.sh` and run the commands below.

#### Build and run the DARP ROS node:

```bash
cd root/workspace
# Build the ROS node
colcon build --packages-select darp_ros2 && source install/setup.bash
# Run the node
ros2 run darp_ros2 simple_planner
```

#### Unit testing:
```bash
ros2 run darp_ros2 coordinate_converter_test
```

#### Run DARP with pygame visualization:
```bash
ros2 run darp_ros2 simple_planner --ros-args -p visualize_darp:=true
```

#### Dev workflow with rviz:

In terminal 1, run the DARP node:
```
cd /home/ubuntu/Documents/ros-task/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run darp_ros2 simple_planner
```

In terminal 2, echo the coverage path topic:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /coverage_path
ros2 topic echo /coverage_path --once
```

In terminal 3, visualize the coverage path topic in rviz:
```
rviz2
```

- Click "Add" button
- Select "By topic" tab
- Find /coverage_path → Path

