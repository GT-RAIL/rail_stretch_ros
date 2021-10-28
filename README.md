# rail_stretch_ros
RAIL ROS Package for Stretch RE1

## Run navigation with Gmapping

Open a terminal and launch the driver

```
roslaunch rail_stretch_config rail_stretch_bringup.launch
```

In a new terminal,run move_base with gmapping

```
roslaunch rail_stretch_navigation navigation.launch
```

## Control joints using set_joint_state service

Open a terminal and launch the driver

```
roslaunch rail_stretch_config rail_stretch_bringup.launch
```

In a new terminal run

```
roslaunch rail_stretch_manipulation joint_control.launch
```

In a new terminal, call the service 

```
rosservice call /joint_control/set_joint_state "joint_name: 'joint_lift'
joint_value: 0.4"
```
Choose one of the following for joint_name

- wrist_extension
- joint_lift
- joint_arm_l3
- joint_arm_l2
- joint_arm_l1
- joint_arm_l0
- joint_head_pan
- joint_head_tilt
- joint_wrist_yaw
- joint_gripper_finger_left
- joint_gripper_finger_right
- joint_mobile_base_translation

