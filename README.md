# ros_occupancygrid
A python program to make ros simulated robot explore the given map using occupancy grid mapping

The starter code is available from https://github.com/DeepBlue14/mcl_ws. Under scripts there is a python file "mapping.py" which makes the robot move around the given and identifies whether the grid/bin is occupied or free . The python code subcribes to "/stage/base_pose_ground_truth" and "/robot/base_scan" topics to get the sensor reading of the robot and the current x,y coordinates and yaw angle of the robot. It publishes the speed and rotation to the topic "/robot/cmd_vel". The ouput is an visual map which shows the areas of the map being occupied or being free.
```sh

After cloning, do the following:
$ cd mcl_ws
$ rm –r devel build
$ catkin_make
$ source devel/setup.bash
$ cd src/no_weights/src/raycaster
$ make clean
$ make
$ cd ../../../with_weights/src/raycaster
$ make clean
$ make
```
To run this simulation on one node(terminal) run "roslaunch uml_mcl mcl.launch" which opens the map and on another node run "rosrun ros_gridlocaliztion mcl.py" which makes the robot move and identifies the areas of the map being occupied or not.
