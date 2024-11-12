# Research Track I - First Assignment

This repository contains the assignment work for the **Research Track** course, done by:  
**Rubin Khadka Chhetri**  
**ID: 6558048**

## Introduction

This repository implements a ROS package containing two main nodes:

- **User Interface node**
- **Distance Monitor node**

These nodes work together within the **`turtlesim`** simulation environment to create a simple, interactive system for controlling and monitoring two turtles.

This assignment was completed using both **Python** and **C++**.

Below are details about each node and its functionality.

## Node Details

### 1. **User Interface Node (`user_interface`)**

The `user_interface` node is responsible for receiving user inputs and controlling the movement of two turtles (`turtle1` and `turtle2`) in the simulator. Its main features include:

- Spawning a second turtle (`turtle2`) in the simulation environment.
- Prompting the user to:
  - Choose which turtle to control (either `turtle1` or `turtle2`).
  - Specify the linear and angular velocities for the selected turtle.
- Sending movement commands to the selected turtle, causing it to move for one second. Afterward, the turtle stops, and the interface is ready for the next command.

This node provides a straightforward way for users to control turtle movements in real-time using a command-line interface.

### 2. **Distance Monitor Node (`distance_monitor`)**

The `distance_monitor` node ensures the turtles maintain safe distances from each other and remain within the boundaries of the simulation area. It continuously monitors and calculates the positions of both turtles, with the following key features:

- Continuously calculating the distance between `turtle1` and `turtle2`, publishing this data on a ROS topic for real-time monitoring.
- Automatically stopping a turtle if it gets too close to the other turtle.
- Stopping a turtle if it moves too close to the simulation boundaries, preventing it from exiting the allowed area.

This node acts as a safety mechanism to prevent collisions and boundary violations, ensuring controlled interactions between the turtles in the simulation environment.
