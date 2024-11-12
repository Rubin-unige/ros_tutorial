# Research Track First Assignment

This is the assignment work for the **Research Track** course,  
done by: <br>
**Rubin Khadka Chhetri**<br>
**6558048**<br>

## Introduction

Repository Contents
This repository includes:

Two ROS nodes:
turtle_user_interface: Allows a user to interact with and control two turtles in the simulation.
turtle_distance_monitor: Continuously monitors and manages the distance between the turtles, ensuring they stay within safe bounds.

### Node Details
1. **User Inteface Node (user_interface)**
This node handles user interactions, allowing control of the two turtles in the simulator (turtle1 and turtle2). Specifically, it:

Spawns a second turtle (turtle2) in the simulator at a specified position.
Prompts the user to select which turtle to control (turtle1 or turtle2).
Takes user input for the turtle's linear and angular velocity.
Sends movement commands to the selected turtle for one second before stopping and prompting for the next command.
This node allows the user to control turtle movements step-by-step through a simple command-line interface.

2. **Distance Monitor Node (distance_monitor)**
This node ensures the turtles maintain safe distances from each other and from the simulator boundaries. It:

Calculates the distance between turtle1 and turtle2 continuously.
Publishes this distance on a dedicated ROS topic for monitoring.
Stops a turtle if it comes too close to the other turtle or approaches the edges of the simulator (pre-defined boundary limits).
This node acts as a safety mechanism, helping to manage the turtles' interactions within the environment.