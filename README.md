
# ERC Hackathon 2024

This repository contains all of the instructions and files required to solve the Electronics and Robotics Club's Hackathon 2024.<br><br>
The hackathon is focused on the design of an **Autonomous Electric Faliure Detection and Repair Robot**. Your team has to provide a solution consisting of the mechanical design, electronics interfacing and an implementation of autonomous behavior. All three tasks are linked to the same final goal and yet can be completed independently.<br><br>
The registration form link can be found [here](https://forms.gle/wjGYFgRjN8xn6po36).<br><br>
The tentative deadline for submission is **11:59 P.M.** on **21st August**<br><br>
In case of any queries contact:<br><br>
Parth Shah: 7021992504<br>
Ritwik Sharma: 9999326476<br>
Ajinkya Deshpande: 9987143576<br>
Vimarsh Shah: 8200096029<br>
Ansh Parmeshwar: 7208771691


#### Teams with complete submissions will be inducted into ERC!
___

## Problem Statement

The detailed problem statement can be found [here](./erc_hackathon_23.pdf)<br><br>
Robotics as a domain can be divided as:<br>
1. Design (Mechanical and Electronics: What components and sensors is the robot made of)
2. Perception (Obtaining information about the robot's environment using the sensors)
3. Planning (Calculating a series of behavious to execute based on what is desired and what the environment looks like)
4. Control (Executing the planned behaviours and ensuring the goal is actually achieved via the robot's actuators)
<br>
All four are closely linked to each other and most autonomous robotic systems will need all four.<br><br>
The hackathon contains atleast one task from each broad divison. However this hackathon is structured such that every task is independent of each other. This will not be the case in a real world robot, but it is a good place to dive into designing a functional robotic system.<br>

___

## Setting up the environment

### Mechanical Design and Analysis
Use [Fusion 360](https://www.autodesk.in/products/fusion-360/education) for CAD modelling and [ANSYS](https://www.youtube.com/playlist?list=PL0Ya8d8RGCTqSaM6GbGHXqAUq1ga7-N__) for structural analysis.

### Electronics Design and Interfacing
The circuit simulation is to be done using the [TinkerCad](https://www.tinkercad.com/) simulator which does not require any installation. PCB design is to be done using [EAGLE](https://www.autodesk.in/products/eagle/overview). EAGLE is a paid software but is free for students (make use of your BITS mail, since you would have already made an autodesk account for EG).<br><br>
**Important Clarification**: Use an **Arduino Uno** along with other required components for the electronics simulation task.
<br>

### Automation (Perception, Planning and Control)
The following are the system requirements for simulating the automation task:
1. Linux (Ubuntu 20.04 [dual boot](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/) recommended. Refer to problem statement for alternatives.)
2. [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu#Ubuntu_install_of_ROS_Noetic)
- It is recommended that you install ```ros-noetic-desktop-full``` as it also installs other required packages along with it
3. Gazebo (Not required separately if you have installed ```ros-noetic-desktop-full```)<br>
To verify, use this command:
```
gazebo --version
```
If you are able to see the version, Gazebo is installed.<br><br>
4. TurtleBot3 [Setup manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) (**NOTE:** Change kinetic to noetic if you follow the instructions in the link)<br>
<br>

#### Detailed setup instructions for turtlebot3: 
Run the following commands sequentially 
<br>
**Install dependent packages:**
```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan \
ros-noetic-rosserial-arduino ros-noetic-rosserial-python \
ros-noetic-rosserial-server ros-noetic-rosserial-client \
ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
ros-noetic-compressed-image-transport ros-noetic-rqt* \
ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```
<br>

**Install turtlebot3:**
```
sudo apt-get install ros-noetic-dynamixel-sdk
```

```
sudo apt-get install ros-noetic-turtlebot3-msgs
```
```
sudo apt-get install ros-noetic-turtlebot3
```
<br>

**Install turtlebot3 simulation package:**<br>
Navigate to your catkin workspace source folder
```
cd ~/catkin_ws/src/
```
Clone the turtlebot3_simulations repo:
```
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Build your workspace (Note: Most resources including the book by Morgan Quigley use catkin_make instead of catkin build, but make sure you use catkin build instead) 
```
cd ~/catkin_ws && catkin build
```
<br><br>
**Automation task package**<br>
Clone this repository in your catkin workspace source folder.
```
cd ~/catkin_ws/src/
```
```
git clone https://github.com/ERC-BPGC/ERC-hackathon-2023.git
```
The ROS package ```robotics_hackathon_automation``` contains all the files required.<br> 
This completes the environment setup for the autonmation task.

