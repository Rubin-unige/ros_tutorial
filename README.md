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

The root of this repository is the package folder. When pulling the repository for the first time, place it directly in the src folder of your workspace.

## Getting started (read before action)

### Prerequisites
Ensure the following packages are installed:
```
sudo apt-get update
sudo apt-get install ros-neotic-turtlesim
```

### Clone the Repository

Create a new workspace:
```
mkdir -p ~/my_new_ws/src
cd ~/my_new_ws/src
```

Clone this repository:
```
git clone https://github.com/Rubin-unige/assignment1_rt.git
```

Source the workspace by adding it to your .bashrc file:
```
echo "source ~/my_new_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Return to the workspace root directory and build the package:
```
cd ~/my_new_ws
catkin_make
```