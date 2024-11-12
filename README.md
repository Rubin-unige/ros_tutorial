# Research Track First Assignment

This is the assignment work for the **Research Track** class,  
done by  
**Rubin Khadka Chhetri**

## Introduction

This repository contains the code for controlling turtles in the ROS `turtlesim` simulator, as part of the **First Assignment** in the Research Track class. The program allows the user to control two turtles (`turtle1` and `turtle2`) by setting their linear and angular velocities through a simple text interface. The turtles will move for a specified duration (1 second) before stopping, and the user can enter new commands to control the turtles again.

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
