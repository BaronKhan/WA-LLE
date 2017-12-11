# HCR-walker
This repo contains a single catkin-style ROS package, and all the ROS nodes should be contained in this package (we clone this repo in the src/ directory our own catkin workspaces).

Current code is just the basic publisher/subscriber example for C++. A similar setup is possible with Python, following [this tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29).

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

## Nodes
- `talker` : Basic subscriber example
- `listener` : Basic publisher example
- `fall_data_collector`: collects fall data from IMU and gait analysis, and writes them to CSV files
- `fall_detector`: determines the falling state of the user
- `fall_detector_trainer`: creates an SVM classification model

#### Fall Detection Requirements
The fall detector nodes need SciPy and sklearn. Install them on the Rapsberry Pi using `sudo apt-get install python-scipy python-sklearn`

## Checklist
- [x] Design report
- [ ] Everything else
