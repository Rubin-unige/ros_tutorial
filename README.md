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
- **`/msg`**: Contains custom message definitions.
  - `turtle_distance.msg`: Custom message file for distance monitoring and boundary status.

- **`/scripts`**: Contains Python scripts used for the nodes in this project. 
  - `user_interface.py`: Python version of user interface node.
  - `distance_monitor.py`: Python version of distance monitor node.

- **`/src`**: Contains C++ source files.
  - `user_interface.cpp`: C++ version of the user interface node.
  - `distance_monitor.cpp`: C++ version of the distance monitor node.

- **`/CMakeLists.txt`**: Specifies the package build rules.

- **`/package.xml`**: Lists dependencies and package metadata.

- **`/README.md`**: Read me file.

## Getting Started (Read Before Action)

### Prerequisites
Before proceeding, make sure that **`ROS Noetic`** is installed on your system.<br>
If you haven’t set up ROS yet, check this official guide to install ROS: <br>
(https://wiki.ros.org/noetic/Installation/Ubuntu) <br>

Additionally, you’ll need **`Python 3`** and **`turtlesim`** package to run this project. Ensure they are installed on your system. If not, you can install them by running:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlesim
sudo apt-get install python3
```
After installation, you can proceed to cloning the repository.

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

To ensure that your workspace is sourced automatically every time you start a new terminal session, add it to your `.bashrc` file:
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
After building, your workspace will be ready to launch the nodes in the package.

## Launch Simulation and Nodes

1. **Start the ROS Master**

Before running any ROS nodes, make sure the ROS Master is up and running. Open a terminal and start `roscore`:
```bash
roscore
```
2. **Run the Turtlesim Node**

Next, start the `Turtlesim` node in a new terminal to launch the simulation environment:
```bash
rosrun turtlesim turtlesim_node
```
This will open the Turtlesim window where the turtles (`turtle1` and `turtle2`) will appear.

3. **Run the User Interface and Distance Monitor Nodes**

At this point, you can proceed to run either the **C++** or **Python** version of the `User Interface` and `Distance Monitor` nodes, depending on which implementation you want to use.

### Running the C++ Version
---
To run the C++ nodes, follow these steps:
- Make sure that the `roscore` and `turtlesim` nodes are running.

- In a new terminal, run the **C++ User Interface Node**:
```bash
rosrun assignment1_rt user_interface
```
- In another terminal, run the **C++ Distance Monitor Node**:
```bash
rosrun assignment1_rt distance_monitor
```
This will start both the **C++ user interface** for controlling the turtles and the **distance monitor** to track their movements.

### Running the Python
---
To run the Python nodes, follow these steps:
- Make sure that the `roscore` and `turtlesim` nodes are running.

- **Make the Python scripts executable**

Before running the Python scripts, you need to ensure they are executable. Run the following command for each Python script (`user_interface.py` and `distance_monitor.py`):
``` bash
chmod +x ~/my_new_ws/src/assignment1_rt/scripts/user_interface.py
chmod +x ~/my_new_ws/src/assignment1_rt/scripts/distance_monitor.py
```

- After making the scripts executable, run the **Python User Interface Node** in the same terminal:
```bash
rosrun assignment1_rt user_interface.py
```
- Open a new terminal and run the **Python Distance Monitor Node**:
```bash
rosrun assignment1_rt distance_monitor.py
```
This will start both the **Python user interface** for controlling the turtles and the **distance monitor** to track their movements.

4. **Stopping the nodes**

To stop the nodes, simply press `Ctrl+C` in the terminal where each node is running (`User Interface`, `Distance Monitor`, `Turtlesim`, or `roscore`). This will terminate the nodes and stop the simulation.

## Implementation
### User Interface node
**Implementation**
- **Spawning Turtle2**

The `user_interface` node automatically spawns a second turtle, `turtle2`, in the simulation when the program starts. This is accomplished using the `/spawn` service provided by `turtlesim`, which allows for creating a new turtle at a specified position and orientation in the simulation environment.

In this implementation:

- `turtle2` is spawned at coordinates **(5.0, 2.0)** with an orientation of **0.0** radians, positioning it to face directly to the right.
- The `/spawn` service is invoked during the initialization phase of the node, ensuring `turtle2` is added automatically without requiring user input.

The relevant parameters and their values are specified as part of the service request, as shown below:
```cpp
  // Initialise service clients
  ros::ServiceClient client_spawn = nh.serviceClient <turtlesim::Spawn> ("/spawn");
  turtlesim::Spawn spawn_srv;
  // Spawn Turtle
  spawn_srv.request.x = 5.0;
  spawn_srv.request.y = 2.0;
  spawn_srv.request.theta = 0.0;
  spawn_srv.request.name = "turtle2";
  client_spawn.call(spawn_srv);
```

- **User Interface**

The core functionality of the user_interface node is to enable real-time interaction between the user and the simulation. The interface is designed to:

Prompt the User:

Select the turtle to control (turtle1 or turtle2).
Enter the desired linear and angular velocities.
Process Input:
User inputs are validated to ensure correctness. If the selected turtle name is invalid or non-numeric values are entered for velocities, the system provides appropriate error messages and prompts the user to re-enter valid data.

Execute Commands:
Based on the validated input, velocity commands are published to the respective turtle’s velocity topic (/turtle1/cmd_vel or /turtle2/cmd_vel). The commands are executed for a duration of one second, after which the turtle stops by publishing a zero-velocity command. This controlled approach ensures that the turtle’s movement is predictable and precise.

Publishing Velocity Commands
The velocity commands for the turtles are represented as geometry_msgs/Twist messages. Each message specifies:

Linear velocity (linear.x) to control forward/backward motion.
Angular velocity (angular.z) to control rotation.
By publishing these commands to the appropriate topics, the turtles respond immediately, allowing for intuitive and dynamic control.

Issues and Solutions
Issues Encountered
Invalid Input Handling:
Users might:
Enter an invalid turtle name (e.g., misspell turtle1 or turtle2).
Provide non-numeric or nonsensical values for velocities.
Such cases previously led to crashes or unexpected behavior in the node.
Solutions
Enhanced Error Handling:
Turtle Selection: If an invalid turtle name is entered, the program displays an error message and prompts the user until a valid name is provided.
Velocity Input: If non-numeric values are entered for linear or angular velocities, the system clears the invalid input and re-prompts the user.
These measures ensure the system remains robust and user-friendly, regardless of erroneous input. As a result, the node can gracefully handle edge cases, enhancing the user experience.

### Distance Monitor node
- Implementation
- Issues
- Solutions