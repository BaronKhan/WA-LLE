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

5. Open a new terminal and run the talker (publisher):
```
rosrun beginner_tutorials talker
```
You should see messages printing on the terminal, corresponding to the message being published.

6. Open a new terminal and run the listener (subscriber):
```
rosrun beginner_tutorials listener
```
You should see messages printing on the terminal, corresponding to the messages received (should be same as those sent by the talker).

## Nodes
- `talker` : Basic subsrciber example
- `listener` : Basic publisher example
- `fall_data_collector`: collects fall data from IMU and gait analysis, and writes them to CSV files
- `fall_detector`: determines the falling state of the user

## Checklist
- [x] Design report
- [ ] Everything else
