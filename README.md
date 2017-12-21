# WA-LLE: Walking Aid - Limitless Living for the Elderly

This repository contains work that has been done to develop robotic walking aids for the elderly.

*This README should always reflect the current state of the repository. If you create a new node, please add it to the Nodes section below with a short description and update the Usage instructions as necessary.*

## Usage
1. Clone this repo/package into your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

2. Make the package: 
```
cd ~/catkin_ws
catkin_make
```

3. Source the catkin setup file:
```
source ./devel/setup.bash
```

4. Run roscore (required before running any ROS applications):
```
roscore
```

5. Run Astra node
```
roslaunch astra_launch astra.launch
```

6. Run OpenPose node and adapter
```
rosrun openpose_ros openpose_ros_node
rosrun hcr_walker hcr_openpose_adapter.py
```

7. Run gait detection and publisher
```
rosrun hcr_walker hcr_gait
```

8. Run IMU and fall detector
```
rosrun hcr_walker arduino_send_node.py
rosrun hcr_walker hcr_imu.py
rosrun hcr_walker fall_detector.py
```

9. Run panic button
```
sudo nodejs src/panicButton/index.js
```

## Nodes
- `arduino_send_node` : USB serial connection between ROS and Arduino
- `hcr_imu` : Connects to an IMU and publishes its gyroscope and accelerometer components
- `hcr_gait` : gait detection node using OpenPose
- `fall_data_collector`: collects fall data from IMU and gait analysis, and writes them to CSV files
- `fall_detector`: determines the falling state of the user
- `fall_detector_trainer`: creates an SVM classification model

#### IMU requirements
The IMU requires bluez and bluepy. Install them using `sudo apt-get install bluez` and `sudo pip install bluepy`.

#### Fall Detection Requirements
The fall detector nodes need SciPy and sklearn in order to use the SVM (diabled by default). Install them using `sudo apt-get install python-scipy python-sklearn`.

## Contributors
- mdhchan
- sabrinalucy96
- liu09222003
- JunShern
- BaronKhan
- JB515