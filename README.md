# Autonomous Stack for WAM-V Boat

<img width="2551" height="1395" alt="Lidar-Camera Fusion" src="https://github.com/user-attachments/assets/cd78e905-6cfc-44cb-b5bf-b66d8d87875d" />


## About Project
This Project is built in accordance with the Virtual RobotX (VRX) environment, official release by the Oper Robotics, is also the "virtual venue" for the [VRX Competition](https://github.com/osrf/vrx/wiki). The project includes:
* Teleoperation for WAM-V boat
* Maritime object detection for obstacles using custom YOLOv8 model (my_model/yolo_modd.pt) with an 0.715 mAP
* Pose estimation for the detected obstacle using early stage Lidar-Camera sensor fusion


## System Requirements
* Ubuntu 20.04
* ROS2 Humble
* Gazebo Garden
* RVIZ2

## Genral Setup
#### -Verify you have ROS2 Humble distribution installed and also CMAKE necessary installations. Refer to [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

On command line run:
```sh
echo $ROS_DISTRO
```
#### -Install previously the following packages and any additional in your linux distribution running on the terminal the command:
```sh 
sudo apt install python3-colcon-common-extensions python3-pip
```

#### -IMPORTANT: Follow the installation procedure carefully to install VRX environment in your local machine from '[HERE](https://github.com/osrf/vrx/wiki/installation_method_tutorial)'!

NOTE: "vrx_ws" will be the VRX environment workspace if you've successfully setup.

#### -Install all necessary dependencies and libraries with pip for insrtance. Recommended to run in your own python environment.

## Steps to build and run project.

### Install python packages 

Check if the below libraries/ module are installed before package setup
* For Obstacle Detection Package:
```sh
sudo apt update
sudo apt upgrade
sudo apt install python3-opencv
pip install ultralytics
pip install "numpy<2"
sudo apt install ros-humble-cv-bridge
sudo apt install libopencv-dev
```
* For Sensor Fusion Package:
```sh
sudo apt install ros-humble-tf-transformations
pip3 install transforms3d
```

### Create the workspace and clone the 'src/' in your worksapce
Create a folder in your system and locate the src pkg
```sh
  mkdir -p ~/auto_ws
  cd ~/auto_ws
```
Clone the repository package inside this folder

```sh
   git clone https://github.com/HamsaavarthanR/wamv_autonomous_ws.git
```

Downlod all the dependencies before runnning a build.
```sh
rosdep install -i --from-path src --rosdistro humble -y
```
Run this command to be at root of your workspace (~/auto_ws) and build your workspace
```sh
colcon build --symlink-install
```


Source ROS2, "vrx_ws" and the current "auto_ws" workspace overlay:

```sh
source install/setup.bash
source ~/(path_to_vrx_workspace)/install/setup.bash
source ~/(path_to_auto_workspace)/install/setup.bash
```


### Launch Gazebo

Run the following command at the root of your "vrx_ws" workspace once you source 

```sh
cd src/vrx/vrx_urdf/
ros2 launch vrx_gz competition.launch.py
```

This will display the WAM-V boat in the 'sydney_regatta' world.


### RVIZ

To launch rviz open a new terminal, source the "vrx_ws" again and run: 

```sh
ros2 launch vrx_gazebo rviz.launch.py
```

To visualize lidar scanner run before in other terminal the gazebo simulation and after launch RVIZ again and use the **Add** button and look for select the **laser scanner** plugin.

* Add Lidar plugin for topic "wamv/sensors/lidars/lidar_wamv_sensor/points" to lidar cloud data.

After check for the scan topic and it should be visible the lidar.

<img width="2548" height="1346" alt="obs_det_3" src="https://github.com/user-attachments/assets/8a4593c7-3529-4320-b2fe-58e5576db0f9" />

## Running scripts 

Inside the src folder of the package, these are the following functionalities you can execute:


### Perception Stack

After having Gazebo and RViz visualisation running, open a new terminal and run the following command after sourcing "auto_ws":

```sh
ros2 launch perception_stack obstacle_awareness.launch.py
```

* Add Lidar plugin for topic "wamv/sensors/cameras/front_left_camera_sensor/obstacle_image" to visualize detected obstacles image.

<img width="2158" height="1056" alt="Screenshot 2025-08-04 at 7 28 46 PM" src="https://github.com/user-attachments/assets/e60e8933-7914-40bc-9358-b405f8895384" />


This will automatically launch 'obstacle_detector' and 'sensor_fusion' nodes to run concurrently using multi-threading.


<img width="2551" height="1395" alt="Lidar-Camera Fusion" src="https://github.com/user-attachments/assets/0883d4c9-d440-487b-828e-1d61516fc948" />

Try repositioning the WAM-V boat at [-517, 190, -0.21] for better visibility:
* Close Gazebo and RViz Terminals
* Find "competition.launch.py" script at 'vrx_ws/src/vrx/vrx_qz/launch
* Make the following edit, shown below, on line number 46
* <img width="1341" height="771" alt="Screenshot 2025-08-04 at 7 14 05 PM" src="https://github.com/user-attachments/assets/b62ba10b-da3a-49d8-b598-cf5b769b783c" />
* Now, 'cd' to the root 'vrx_ws' directory and perform 'colcon_build'
* Relaunch Gazebo and RViz again as usual.

#### Launch Perception stack to obtain estimated pose of the detected obstacles with respect to the front left camera frame.


### Teleoperation:

Control WAM-V boat using keyboard keys:
* W - Move Forward
* S - Move Backward
* D - Towards Right
* A - Towards Left
* Q - Quit and shutdown node

Open a new terminal, source "auto_ws" and run:

```sh
  ros2 run teleop_ctrl teleop_keyboard
```

<img width="2553" height="1411" alt="Cam-Lidar RViz" src="https://github.com/user-attachments/assets/4e4594d6-87d2-47bd-bae6-5b12115255f4" />


