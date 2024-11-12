# Research Track First Assignment

This is the assignment work for the **Research Track** course,  
done by: <br>
**Rubin Khadka Chhetri**
**6558048**

## Introduction

This repository contains the code for the First Assignment of the Research Track I course, designed to provide hands-on experience with ROS (Robot Operating System) using the turtlesim simulator. The assignment is structured around creating a ROS package named assignment1_rt, which includes two nodes with distinct functionalities: a User Interface (UI) node and a Distance Monitoring node.

1. **Node 1: User Interface (UI)**: The UI node allows users to control one of the two turtles in the simulation (turtle1 or turtle2).
- This node:   Spawns a new turtle, named turtle2, in the simulation environment.
Provides a simple text-based interface to accept user commands. Users can select which turtle they want to control (turtle1 or turtle2) and specify the turtle's linear and angular velocities.
Sends movement commands to the selected turtle for a duration of 1 second, after which the turtle stops. The user can then input a new command to control the turtle again.

2. **Node 2: Distance Monitoring**
The Distance node is responsible for monitoring the position of the turtles relative to each other and to the environment boundaries. 
- This node:   Continuously checks the distance between turtle1 and turtle2 and publishes this distance to a dedicated topic using std_msgs/Float32.
Ensures safety by stopping a moving turtle if the turtles come too close to each other or if a turtle gets too close to the boundary limits (e.g., if x or y coordinates are greater than 10.0 or less than 1.0).

## Repository Structure

The root of this repository is the **ROS package** folder. After cloning the repository, you should place it inside the `src` folder of your ROS workspace.

### Directory Structure:



## Getting Started

Follow these steps to get your environment ready for running the ROS program.

### Prerequisites

1. **ROS Installation**: Ensure that you have **ROS** installed on your system (this code has been tested with **ROS Noetic**).
   - Follow the instructions on the [ROS website](http://www.ros.org) for installing ROS.
   
2. **Install Dependencies**:
   - Install the `turtlesim` package if you don't have it:
     ```bash
     sudo apt-get install ros-noetic-turtlesim
     ```

### Clone the Repository

Create a new workspace and clone the repository into the `src` folder:

```bash
mkdir -p ~/my_new_ws/src
cd ~/my_new_ws/src
git clone https://github.com/Rubin-unige/assignment1_rt.git
