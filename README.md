# Robotics Project

This package requires to be inside the same project of [Locosim](https://github.com/mfocchi/locosim).

## ROS Nodes
We have two ROS nodes: `movement_handler` and `main`.

### Main
This node interacts with the **UR5** and the other modules, as of now only represented by the **movement handler**. 

### Movement Handler
This node provides a service that computes the movements that the **UR5** should do.

## Building
Before starting to build the project, you need to include the Eigen submodule in the git repo
```
git submodule update --init --recursive
```
To build the project simply run the Makefile (do not worry about sourcing the project since we already integrated this step inside the Makefile). 
```
make
```

## Running
The following steps are sequential and need to be executed in different terminals.

### Run the robot
In order to run the simulation of the robot, execute
```
make run-robot
```

Running the robot closes every instance of **rosmaster** and opens a new one, so it is not necessary to run the **roscore** in a seperate terminal.

Inside another terminal you have to execute
```
make position-blocks
```
So that the blocks get positioned in the table. Execute `make delete-blocks` to remove them.

### Start the Movement Handler server
Execute
```
make run-movement
```
This starts a server on the current console, given that the following command has to be executed in a different one.

### Execute the Main
Execute
```
make run-client
```
At the moment the **movement handler** (launched with `make run-movement`) provides only dummy data, so the actions of the robot are not planned and are randomic.