# Robotics Project

This package requires to be inside the same project of [Locosim](https://github.com/mfocchi/locosim).

## ROS Nodes
We have two ROS nodes: `movement_handler` and `main`.

### Main
This node interacts with the **UR5** and the other modules, as of now only represented by the **movement handler**. 

### Movement Handler
This node provides a service that computes the movements that the **UR5** should do.

## Building
You can directly build the project.

If you want to build only this pacakge you can execute the file `build_package.sh`, this one executes the following command

```
catkin_make --pkg robotics_project
```

## Running
The following steps are sequential.

### Run the robot
In order to run the simulation of the robot, execute
```
python3 -i $LOCOSIM_DIR/robot_control/base_controllers/ur5_generic.py
```

Running the robot closes every instance of **rosmaster** and opens a new one, so it isn't necessary to run the **roscore** in a seperate terminal.

### Start the Movement Handler server
Execute
```
rosrun robotics_project movement_handler
```
This starts a server on the current console, given that the following command has to be executed in a different one.

### Execute the Main
Execute
```
rosrun robotics_project main
```
At the moment the **movement handler** provides only dummy data, so the actions of the robot are not planned and are randomic.
