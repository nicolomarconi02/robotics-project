# Robotics Project

The project consists of the implementation of a system to recognise the position of blocks placed on a table, and move them using an anthropomorphic arm manipulator.

Here a video of our robot working:

https://github.com/user-attachments/assets/abd76676-b3e1-43fc-a70d-b7380857d6b2

If you are interested in how we developed the project, have a look at the [report](https://github.com/nicolomarconi02/robotics-project/blob/main/assignment/Robotics_Project.pdf).

# How to install

This package requires to be inside the same ROS workspace as the project of [Locosim](https://github.com/mfocchi/locosim). Once Locosim is installed and this repo has been cloned, you may have to change the 5th line in the Makefile so that the path to the workspace matches. By default, we set
```
WORKSPACE_PATH=~/ros_ws
```

> **Disclaimer:** The [Locosim](https://github.com/mfocchi/locosim) repository has been updated since we installed it. We developed the project based on this [version](https://github.com/mfocchi/locosim/commit/69563be995b5f980840afed223e1f13d7cf07cfa) (commit 1bb4e0702ee3a260c74d750cfad8f09de96a6087).


## ROS Nodes
We have three ROS nodes:

* `client` : This node interacts with the **UR5** and the other modules
* `movement_handler` : This node provides a service that computes the movements that the **UR5** should do
* `vision` : This node provides a service that detects the blocks' pose on the table and returns their informations to the client module

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
This starts the vision module on the current console.

### Execute the Client module
Execute
```
make run-client
```
If every module is started correctly, the robot will begin to move when the vision module has computed the blocks' positions and the movement module has returned the path that the robot has to follow.

## Documentation
To generate Doxygen documentation, you need to have Doxygen installed on your computer. If you haven’t installed it yet, you can do so with:
```
sudo apt install -y doxygen
```

Then run this command to generate the documentation of the project
```
make documentation
```
You will have to launch the html file in `/docs/html/index.html` to explore the documentation of the project.
