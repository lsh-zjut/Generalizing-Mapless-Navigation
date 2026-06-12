# sim_world Workspace

`sim_world workspace` is a ROS Noetic simulation workspace for Gazebo. In this repository it works together with the sibling `deployment/` folder, which provides the visual navigation and local obstacle-avoidance nodes.

In this setup:

- `sim_world workspace` provides Gazebo worlds, the Jackal robot description, camera topics, LiDAR topics, odometry, and launch files
- `deployment/` provides the visual navigation model, topomap assets, and navigation logic
- the simulator and deployment folders are intended to run together for simulated visual navigation in `sim_world`

## Workspace Overview

This workspace mainly contains:

- `src/sim_world/`: simulation worlds, launch files, RViz configs, and helper scripts
- `src/jackal_description/`: Jackal robot description, sensors, meshes, and Gazebo configuration

The `sim_world` package currently includes two simulation environments:

- `world0.launch` / `sim0.world`: route for `sim_world0`
- `world1.launch` / `sim1.world`: route for `sim_world1`

These worlds are commonly used together with the topomaps and navigation scripts in `../deployment`.

## How It Connects To `deployment`

This repository is meant to serve as the simulator side of the full navigation pipeline.

Typical responsibility split:

- `sim_world workspace`: launches Gazebo and publishes robot sensor data
- `deployment/`: subscribes to camera, LiDAR, and odometry topics and performs visual navigation

The main runtime scripts are:

- `deployment/src/navigate.py`
- `deployment/src/obstacle_avoid.py`

So the overall workflow is:

1. Launch a world from `sim_world workspace`
2. Start the visual navigation stack from `deployment/`
3. Let the robot navigate in Gazebo using the sensor streams provided by this workspace

## Requirements

- Ubuntu 20.04
- ROS Noetic
- Gazebo
- Python 3 tools:

```bash
sudo apt-get install python3-rosdep python3-empy python3-vcstool build-essential
```

Initialize `rosdep` once if needed:

```bash
sudo rosdep init
rosdep update
```

## Clone And Build

Build the workspace from this folder:

```bash
cd sim_world
```

Install dependencies and build:

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Running The Simulation

Before launching Gazebo, place the required Gazebo models under `~/.gazebo/`. Otherwise Gazebo may open with an empty or black scene.

Model download:

- https://drive.google.com/drive/folders/15ZlNQRygDhuBKT8wAKXB_oRsIn1Kpv4V

Launch the indoor world:

```bash
roslaunch sim_world world0.launch
```

Launch the outdoor world:

```bash
roslaunch sim_world world1.launch
```

Launch RViz with manual control tools:

```bash
roslaunch sim_world manual.launch
```

## Typical Integration With `deployment`

After starting one of the worlds in this workspace, run the visual navigation stack from the sibling `deployment/` folder in the same repository.

Make sure the topic names in `deployment/` match the topics published by this workspace, especially:

- `/cmd_vel`
- `/front/scan`
- `/front/rgb/image_raw`
- `/front/depth/image_raw`
- `/front/depth/points`
- `/jackal_velocity_controller/odom`
- `odometry/filtered`

Then use the appropriate topomap in `deployment/`:

- `route0` for `sim_world0`
- `route1` for `sim_world1`

## Gazebo GUI In Containers

`world0.launch` and `world1.launch` are configured to start Gazebo with GUI enabled by default.

`gazebo_ros_control` is currently disabled by default because the current Jackal control plugin chain may cause the robot model to become invalid in Gazebo and disappear from the scene.

If you need to force the control chain back on:

```bash
export JACKAL_ENABLE_GAZEBO_ROS_CONTROL=1
```

If Gazebo GUI does not appear in Docker, X11 forwarding is usually missing. A typical container launch looks like:

```bash
xhost +local:root
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  <your_image> bash
```

Inside the container:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sim_world world0.launch
```

If needed, you can explicitly set:

```bash
roslaunch sim_world world0.launch gui:=true headless:=false
```

## Useful Topics

- Velocity command: `/cmd_vel`
- Internal velocity controller command: `/jackal_velocity_controller/cmd_vel`
- Odometry: `/jackal_velocity_controller/odom`
- Filtered odometry: `odometry/filtered`
- Joint states: `/joint_states`
- Laser scan: `/front/scan`
- RGB image: `/front/rgb/image_raw`
- Depth image: `/front/depth/image_raw`
- Point cloud: `/front/depth/points`

## Camera Configuration

The default depth camera parameters are located in:

- `src/jackal_description/urdf/accessories/kinect.urdf.xacro`

Important parameters include:

- `<horizontal_fov>` for camera field of view
- `<clip><near>` and `<clip><far>` for min/max range

If you use other cameras such as pointgrey or flea3, check:

- `src/jackal_description/urdf/accessories.urdf.xacro`

## Common Maintenance

Clean and rebuild:

```bash
rm -rf build devel
catkin_make
```

Remember to source the workspace each time you open a new terminal:

```bash
source devel/setup.bash
```
