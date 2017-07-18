# tiago_ltl_flexbe

Integration of TIAGo simulation environment with LTL motion planner and FlexBE real-time execution framework

## Description

This package is based on the following three sources:

* TIAGo simulation repo: http://wiki.ros.org/Robots/TIAGo/Tutorials

* LTL motion planner: https://github.com/MengGuo/P_MAS_TG

* FlexBE framework: http://wiki.ros.org/flexbe


## Utilization

+ launch gazebo with the small_office world : 

```
roslaunch tiago_ltl_flexbe pick_test.launch rviz:=false world:=small_office
```
+ Send the robot the mission to pick an object generated randomly on a table :

```
./pick_test.py example
```
The name of the data recording file is passed as an argument and example.txt is stored in the pick_test folder

## Usage 

- The robot model as finite transition system (FTS) is specified in [\[robot_fts.py\]](https://github.com/MengGuo/tiago_ltl_flexbe/blob/master/src/robot_fts.py):

```python
robot_model = [robot_motion, init_pose, robot_action]

```

   where the robot motion and action abstractions are given as [FTS models](https://github.com/MengGuo/tiago_ltl_flexbe/blob/master/src/ltl_tools/ts.py). 


  <p align="center">  
  <img src="https://github.com/MengGuo/tiago_ltl_flexbe/blob/master/src/figures/tiago_ltl_flexbe.png" width="800"/>
  </p>

- Simulation [video](https://vimeo.com/215800825) for different LTL tasks (with only navigation tasks)

- Incorporating TIAGo pre-programmed actions from [\[play_motion\]](http://wiki.ros.org/Robots/TIAGo/Tutorials/motions/play_motion), such as
```
close_gripper
close_gripper_half
do_weights
head_tour
home
offer_gripper
open_gripper
pick_from_floor
pregrasp_weight
reach_floor
reach_max
shake_hands
show_manip_ws
unfold_arm
wave
```

Thus a task can involve any regions of interest in the workspace and any [play_motion] action above,
```python
# python ltl_planner.py '([]<> (r2 && shake_hands)) && ([]<> (r3 && head_tour))'
python ltl_planner.py '<> ((r2 && pick_from_floor) && <> (r3 && reach_max))
```

see [video](https://vimeo.com/218766393) for example.

