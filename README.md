# Food Delivery bot


## Overview

This ROS 2-based Food delivery Bot project implements a behavior tree for an autonomous delivery robot, handling order processing, navigation, and task execution. It subscribes to robot state updates, processes orders, navigates between predefined locations (e.g., kitchen, tables, home), and manages failure conditions such as order cancellations and timeouts.

## [Demonstration](https://drive.google.com/file/d/1Emu8EpixYeHzUSP-oKnuVpvai1Fv_Ksk/view?usp=sharing): 



## Steps to run


```
1. Clone the repo 
```
git clone https://github.com/nivednivu1997/Food-delivery-Bot.git
```
3. Install Dependencies
```
rosdep install --from-path src --ignore-src -y
```
4. Building
```
 colon build --symlink-install
```


## Steps to run 

1. Launch the gazebo simulation
```
ros2 launch butlerbot_rmf_gazebo simulation.launch.py
```
2. Launch the navigation2 stack 
```
ros2 launch butlerbot_navigation navigation.launch.py
```
3. Run food delivery python behaviour tree 
```
cd ROS-Assignment/src &&
python3 py_tree.py
```
4. Run the publisher to give command(example command)

ros2 topic pub --once /robot_state std_msgs/String '{"data": "{\"order_received\": true, \"order_canceled\": false, \"confirmation_kitchen\": true, \"confirmation_table\": {\"table1\": true}, \"current_task\": null, \"tables\": [\"table1\"]}"}'

```




