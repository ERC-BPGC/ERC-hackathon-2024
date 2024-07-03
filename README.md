
# ERC Hackathon 2024

This repository contains all of the instructions and files required to solve the Electronics and Robotics Club's Hackathon 2024.<br><br>
The hackathon is focused on the design of an **Autonomous Electric Faliure Detection and Repair Robot**. Your team has to provide a solution consisting of the mechanical design, electronics interfacing and an implementation of autonomous behavior. All three tasks are linked to the same final goal and yet can be completed independently.<br><br>
The registration form link can be found [here](https://forms.gle/GN1YJnDBnHGvpFKa6).<br><br>

Link to Instructions can be found [here (latest)](https://docs.google.com/document/d/1SDBllctevOAIbVfmJKIbaY46uG1-VVng/edit?usp=sharing&ouid=102608550481394760401&rtpof=true&sd=true).
The deadline for submission is **11:59 P.M.** on **1st August**<br><br>


In case of any queries contact:<br><br>
Parth Shah: 7021992504<br>
Ritwik Sharma: 9999326476<br>
Ajinkya Deshpande: 9987143576<br>
Vimarsh Shah: 8200096029<br>
Ansh Parmeshwar: 7208771691




### Teams/Individuals with complete submissions will be inducted into ERC!
___

## Problem Statement

The detailed problem statement can be found [here](erc_hackathon_24.pdf)<br><br>

Robotics as a domain can be divided as:<br>
1. Design (Mechanical and Electronics: What components and sensors is the robot made of)
2. Perception (Obtaining information about the robot's environment using the sensors)
3. Planning (Calculating a series of behavious to execute based on what is desired and what the environment looks like)
4. Control (Executing the planned behaviours and ensuring the goal is actually achieved via the robot's actuators)
<br>
All four are closely linked to each other and most autonomous robotic systems will need all four.<br><br>
The hackathon contains atleast one task from each broad divison. However this hackathon is structured such that every task is *independent of each other*. This will not be the case in a real world robot, but it is a good place to dive into designing a functional robotic system.<br>

___

## Setting up the environment

### Mechanical Design and Analysis
Use [Fusion 360](https://www.autodesk.in/products/fusion-360/education) for CAD modelling and [ANSYS](https://www.youtube.com/playlist?list=PL0Ya8d8RGCTqSaM6GbGHXqAUq1ga7-N__) for structural analysis.

### Electronics Design and Interfacing
The circuit simulation is to be done using the [TinkerCad](https://www.tinkercad.com/) simulator which does not require any installation. PCB design is to be done using [KiCad](https://www.kicad.org/). KiCad is a free to use and open source software.<br><br>

**Important Clarification**: Use an **Arduino Uno** along with other required components for the electronics simulation task.
<br>

### Automation (Perception, Planning and Control)

We will be using ROS2 for the automation task. It is recomended to use ROS2 - Humble as a standard ROS version for this task.

Follow the complete instructions from [here](ROS_Setup.md).

The ROS package ```hackathon_automation``` contains all the files required for this task<br> 
