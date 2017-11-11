# HCR-walker
I'm thinking the top-level directory of this repo should just be a single package, and all the ROS nodes should be contained in this package (we clone this repo in the src/ directory our own catkin workspaces).

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

## Checklist
- [x] Design report
- [ ] Everything else
