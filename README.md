# Generalizing Mapless Navigation

`Generalizing Mapless Navigation` is a deployment-focused repository for visual obstacle-avoidance navigation in the `sim_world` Gazebo simulator.

This repository keeps only the parts needed to run inference in simulation:

- two simulation environments: `sim_world0` and `sim_world1`
- the visual topological navigation node
- the local obstacle-avoidance node
- two ready-to-run topomaps: `route0` and `route1`

Training code is intentionally not included. The navigation model is used only for inference.

## Repository Layout

```text
Generalizing Mapless Navigation/
|-- deployment/
|   |-- config/
|   |-- model_weights/
|   |-- src/
|   `-- topomaps/
|-- sim_world/
|-- LICENSE
`-- README.md
```

## Runtime Pipeline

1. Launch one of the Gazebo worlds from `sim_world`.
2. Run `deployment/src/navigate.py` to localize against a topomap and predict a waypoint from camera images.
3. Run `deployment/src/obstacle_avoid.py` to convert the waypoint into a safe velocity command using LiDAR and odometry.
4. The robot navigates in simulation with visual guidance plus local obstacle avoidance.

The provided routes are:

| Gazebo world | Launch file | Topomap |
| --- | --- | --- |
| `sim_world0` | `world0.launch` | `deployment/topomaps/images/route0` |
| `sim_world1` | `world1.launch` | `deployment/topomaps/images/route1` |

## Requirements

- Ubuntu 20.04
- ROS Noetic
- Gazebo
- Conda
- Python 3.8
- CUDA-capable GPU recommended for inference

Install ROS build tools if needed:

```bash
sudo apt-get install python3-rosdep python3-empy python3-vcstool build-essential
sudo rosdep init
rosdep update
```

## 1. Build The Simulator Workspace

```bash
cd sim_world
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

Before launching Gazebo, place the required Gazebo models under `~/.gazebo/`.

Model download:

- https://drive.google.com/drive/folders/15ZlNQRygDhuBKT8wAKXB_oRsIn1Kpv4V

## 2. Create The Deployment Environment

```bash
cd deployment
conda env create -f deployment_environment.yaml
conda activate vint_deployment
```

## 3. Place Model Weights

Put the pretrained GNM checkpoint here:

```text
deployment/model_weights/gnm.pth
```

The default config is already set in:

- `deployment/config/models.yaml`
- `deployment/config/gnm.yaml`

Large weight files are not committed to GitHub.

## 4. Run Visual Navigation In `sim_world0`

Terminal 1:

```bash
cd sim_world
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sim_world world0.launch
```

Terminal 2:

```bash
cd deployment/src
conda activate vint_deployment
python navigate.py --model gnm --dir route0 --goal-node -1 --close-threshold 3 --radius 4
```

Terminal 3:

```bash
cd deployment/src
conda activate vint_deployment
python obstacle_avoid.py
```

## 5. Run Visual Navigation In `sim_world1`

Terminal 1:

```bash
cd sim_world
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch sim_world world1.launch
```

Terminal 2:

```bash
cd deployment/src
conda activate vint_deployment
python navigate.py --model gnm --dir route1 --goal-node -1 --close-threshold 3 --radius 4
```

Terminal 3:

```bash
cd deployment/src
conda activate vint_deployment
python obstacle_avoid.py
```

## Default ROS Topics

- RGB image: `/front/rgb/image_raw`
- LiDAR: `/front/scan`
- Odometry: `/gazebo/ground_truth/state`
- Velocity command: `/cmd_vel`

`navigate.py` publishes:

- `/waypoint`
- `/topoplan/reached_goal`

## Create A New Topomap

```bash
cd deployment/src
conda activate vint_deployment
python create_topomap.py --dir my_route --dt 1.0
```

Images will be saved to:

```text
deployment/topomaps/images/my_route
```

Then run navigation with:

```bash
python navigate.py --model gnm --dir my_route
```

## Notes

- This repository is for simulation deployment, not for training.
- The default deployment path uses the GNM model.
- `route0` matches `world0.launch`, and `route1` matches `world1.launch`.
- If Gazebo opens to a black or empty scene, the Gazebo model assets are usually missing from `~/.gazebo/`.

## License

See `LICENSE` and `THIRD_PARTY_NOTICES.md`.
