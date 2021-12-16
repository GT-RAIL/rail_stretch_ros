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

## Control robot using voice input

Open a terminal and launch

```
roslaunch rail_stretch_config rail_stretch_aruco_nav_demo.launch
```

Move around using 'i' , 'j', 'l' and ',' keys to detect aruco markers  

Call ROS service to to take voice input using  

```
rosservice call /stretch_voice
```

For example, "Bring the box from tall table to kitchen counter"

"Bring the cube from kitchen counter to tall table"

To change the objects and corresponding Aruco ID, update rail_stretch_manipulation/config/aruco_marker_dict.yaml

To directly command the robot to go from a starting aruco to ending aruco (without voice), open a terminal and run

```
rosservice call /stretch_mission "from_aruco_name: 'tall table'
object_name: 'cube'
to_aruco_name: 'kitchen counter'"
```
