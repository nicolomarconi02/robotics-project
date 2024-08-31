# Robotics Project

This package requires to be inside the same project of [Locosim](https://github.com/mfocchi/locosim).

## ROS Nodes
We have three ROS nodes: `client`, `movement_handler` and `vision`.

### Client
This node interacts with the **UR5** and the other modules. 

### Movement Handler
This node provides a service that computes the movements that the **UR5** should do.

### Vision
This node provides a service that detects the blocks' pose on the table and returns their informations to the client module

## Building
Before starting to build the project, you need to include the Eigen submodule in the git repo
```
git submodule update --init --recursive
```
To build the project simply run the Makefile (do not worry about sourcing the project since we already integrated this step inside the Makefile). 
```
make
```

Run this command to install all the dependencies needed by the vision module
```
python3 -m pip install -r requirements.txt
```

## Running
The following steps are sequential and need to be executed in different terminals.

### Run the robot
In order to run the simulation of the robot, execute
```
make run-robot
```

Running the robot closes every instance of **rosmaster** and opens a new one, so it is not necessary to run the **roscore** in a seperate terminal.

### Start the Movement Handler module
Execute
```
make run-movement
```
This starts a module on the current console, given that the following command has to be executed in a different one.

### Start the Vision module
When the robot has completed his homing procedure and the blocks are spawned on the table, execute
```
make run-vision
```
This starts the vision module on the current console

### Execute the Client module
Execute
```
make run-client
```
If every module is started correctly, the robot will begin to move when the vision module has computed the blocks' positions and the movement module has returned the path that the robot has to follow

## Documentation
In order to generate doxygen documentation you have to install doxygen
```
make documentation
```
sudo apt install -y doxygen
```
Then run this command to generate the documentation of the project
make documentation
```
You will have to launch the html file in `/docs/html/index.html`
