# Research Track First Assignment

This is the assignment work for the **Research Track** course, done by: <br>
**Rubin Khadka Chhetri**<br>
**6558048**<br>

## Introduction

This repository contains two main ROS nodes: the **User Interface node** and the **Distance Monitor node**. These nodes work together to provide a simple but interactive control and monitoring system for two turtles in the turtlesim simulation environment. Below are the details of each node: <br>

### Node Details

1. **User Interface Node (user_interface)**

This node is responsible for handling user input and controlling the movements of two turtles (turtle1 and turtle2) in the simulator. Its key functions include:<br>

-  Creates a second turtle (turtle2) in the simulation environment at a predefined position.
-  Prompts the user to:
   -  Select which turtle to control (either turtle1 or turtle2).
   -  Set the turtle's linear and angular velocities.
-  Sends movement commands to the selected turtle, causing it to move for one second. After the movement, the turtle stops, and the interface is ready to accept the next command, allowing for step-by-step control.

This node provides a simple and interactive way for users to control turtle movements in real-time through the command line.

2. **Distance Monitor Node (distance_monitor)**

The Distance Monitor node ensures that the turtles maintain safe distances from each other and stay within the boundaries of the simulator. This node continuously calculates and monitors turtle positions. Its key features are:<br>

-  Continuously calculates the distance between turtle1 and turtle2 and publishes this information on a dedicated ROS topic for monitoring.
-  Automatically stops a turtle if it approaches the other turtle within a specified safety threshold.
-  Stops a turtle if it nears the simulator's boundaries (with pre-defined limits, e.g., when x or y coordinates are outside the range of 1.0 to 10.0).

This node acts as a safety mechanism to manage turtle interactions and prevent them from colliding or moving out of bounds.