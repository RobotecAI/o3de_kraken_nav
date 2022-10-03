# Apple kraken navigation #

## Installation ##

It is assumed that ROS2 `galactic` is used and the workspace dir is in `~/o3de_kraken_ws`.

1. Source ROS2

```bash
source /opt/ros/galactic/setup.bash
```

2. Put this package in some workspace directory, inside `src`

```bash
mkdir -p ~/o3de_kraken_ws/src && cd ~/o3de_kraken_ws/src
git clone https://github.com/RobotecAI/o3de_kraken_nav.git
```

3. Go to workspace dir and build this package

```bash
cd ~/o3de_kraken_ws
colcon build 
```

## Running scene

1. Use the [roscon_2022](https://github.com/aws-lumberyard-dev/o3de/tree/roscon_2022)  branch of the `O3DE`.
1. Use the the [pjaroszek/ackermann_drive_model](https://github.com/RobotecAI/o3de-ros2-gem/tree/pjaroszek/ackermann_drive_model) branch of the `o3de-ros-gem`.
1. Use [mp/deappletreeized_orchad_kraken](https://github.com/aws-lumberyard/ROSConDemo/tree/mp/deappletreeized_orchad_kraken) branch of the `ROSConDemo`.

## Running simulation

1. Build and run the `ROSConDemo`
1. Load level `Main`

## Running nav stack

1. Source the workspace

```bash
cd ~/o3de_kraken_ws
source ./install/setup.bash
```

2. (This step is going to be removed) Adjust behavior tree setting: in file `src/o3de_kraken_nav/launch/config/navigation_params.yaml` modify `default_nav_to_pose_bt_xml` (line 6) providing full path to the `bt.xml` file: `/home/$USER/o3de_kraken_ws/src/o3de_kraken_nav/launch/config/bt.xml`
3. Run the navigation stack

```bash
ros2 launch o3de_kraken_nav navigation.launch.py
```

