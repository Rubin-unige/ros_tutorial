# Research Track I - First Assignment

This repository contains the assignment work for the **Research Track** course, completed by:  
**Rubin Khadka Chhetri**  
**ID: 6558048**

## Introduction

This repository implements a ROS package containing two main nodes: 

-  **User Interface node**
-  **Distance Monitor node**

These nodes work together within the **`turtlesim`** simulation environment to create a simple, interactive system for controlling and monitoring two turtles.

This assignment was completed using both **Python** and **C++**.

Below are details about each node and its functionality.<br>

## Node Details

1. **User Interface Node** (`user_interface`)

This node is responsible for handling user input and controlling the movements of two turtles (`turtle1` and `turtle2`) in the simulator. Its key functions include:<br>

-  Spawns a second turtle (`turtle2`) in the simulation environment.
-  Prompts the user to:
   -  Select which turtle to control (either `turtle1` or `turtle2`).
   -  Set the selected turtle's linear and angular velocities.
-  Sends movement commands to the selected turtle, causing it to move for one second. After the movement, the turtle stops, and the interface is ready to accept the next command.

2. **Distance Monitor Node** (`distance_monitor`)

This node ensures that the turtles maintain safe distances from each other and stay within the boundaries of the simulation environment. This node continuously calculates and monitors turtle positions. Its key features are:<br>

-  Continuously calculates the distance between `turtle1` and `turtle2` and publishes this information on a dedicated ROS topic for monitoring.
-  Automatically stops a turtle if it approaches the other turtle.
-  Stops a turtle if it's position is too close to the boundaries.

## Repository Structure

The root of this repository is the package folder, which contains all necessary files and scripts for running the assignment nodes. When cloning the repository for the first time, place it directly in the `src` folder of your ROS workspace.

### Folder and File Overview

- **`/scripts`**: Contains Python scripts used for the nodes in this project. 
  - `user_interface.py`: Implements the user interface node, which enables user control of turtle movement.
  - `distance_monitor.py`: Implements the distance monitor node, which ensures safe distances between turtles and enforces boundary limits.

- **`/src`**: Holds C++ source files.
  - `user_interface.cpp`: C++ version of the user interface node.
  - `distance_monitor.cpp`: C++ version of the distance monitor node.

- **`/CMakeLists.txt`** and **`package.xml`**: Configuration files for building and managing dependencies in the ROS package. 
  - `CMakeLists.txt`: Specifies the package build rules.
  - `package.xml`: Lists dependencies and package metadata.

## Getting Started (Read Before Action)

### Prerequisites

Before proceeding, make sure that **ROS Noetic** is installed on your system.<br>
If you haven’t set up ROS yet, check this website to install ROS: <br>
(https://wiki.ros.org/noetic/Installation/Ubuntu) <br>

Additionally, you’ll need the **`turtlesim`** package to run this project. Install it by running:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlesim
```
then you can proceed to cloning the repository.

### Clone the Repository

1. **Set up your ROS workspace**

Create a new workspace (or use an existing one) and navigate to its `src` directory:
```bash
mkdir -p ~/my_new_ws/src
cd ~/my_new_ws/src
```

2. **Clone this repository**

Clone the assignment repository into your workspace’s `src` folder:
```bash
git clone https://github.com/Rubin-unige/assignment1_rt.git
```
3. **Add the Workspace to Your ROS Environment**

To ensure your workspace is sourced every time a new terminal session starts, add it to your `.bashrc` file:
```bash
echo "source ~/my_new_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

4. **Build the Package**

Navigate to the root of your workspace and build the package using `catkin_make`:
```bash
cd ~/my_new_ws
catkin_make
```

After building, your workspace should be ready to launch the nodes in the package.

## Run 

1. **Start the ROS Master**
Before running any ROS nodes, make sure the ROS Master is up and running. Open a terminal and start roscore:

```bash
roscore
```

This command will start the ROS master, which is responsible for managing the communication between the nodes. You should leave this terminal open and running.

2. **Run the Turtlesim Node**
The turtlesim node provides the simulation environment with the two turtles (turtle1 and turtle2). Open a new terminal and run:

```bash
rosrun turtlesim turtlesim_node
```
This will open the turtlesim window where the turtles will appear.

3. **Run the User Interface and Distance Monitor Nodes**
At this point, you can proceed to either run the C++ version or the Python version of the User Interface and Distance Monitor nodes.

- Running the C++ Version
To run the C++ nodes (both the User Interface and Distance Monitor nodes), follow these steps:

  - Run the C++ User Interface Node:

```bash
rosrun assignment1_rt user_interface
```
This will launch the C++ User Interface node. Follow the prompts to select the turtle and enter velocity commands.

Run the C++ Distance Monitor Node:

Open another terminal and run:

```bash
rosrun assignment1_rt distance_monitor
```
This node will continuously monitor the distances between turtle1 and turtle2, stopping them if necessary.

- Running the Python Version
To run the Python nodes (both the User Interface and Distance Monitor nodes), follow these steps:

Run the Python User Interface Node:

In the terminal, run:

bash
Copy code
rosrun assignment1_rt user_interface.py
This will launch the Python User Interface node. Follow the prompts to select the turtle and enter velocity commands.

Run the Python Distance Monitor Node:

In a new terminal, run:

bash
Copy code
rosrun assignment1_rt distance_monitor.py
This node will continuously monitor the distances between turtle1 and turtle2 and stop them if needed.

