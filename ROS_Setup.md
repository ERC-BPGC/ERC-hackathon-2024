# Instructions to set up ROS2 + Gazebo + Turtlebot3


Instructions to setup your environment is divided into following broad sections:

1. Setting up the environment itself (Installing Ubuntu 22 or equivalent)
2. Setting up ROS2 (we recommend using Humble)
3. Setting up Turtlebot3 and its other packages

===


## Setting up Your environemnt

#### Recommended

We recommend to use Ubuntu 22 - you do not need to remove Windows. You can dual boot Ubuntu 22.04 by [following these instructions](https://www.xda-developers.com/dual-boot-windows-11-linux/). You can download the Ubuntu 22 Desktop image from [here](https://releases.ubuntu.com/jammy/).

#### Altenative (using Windows Sunsystem for Linux)

This approach uses [Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/about) to run any Linux distro on your windows machine itself.

Open Powershell on your windows machine and run
```
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

To determine if WSL 2 is enabled on your machine using both PowerShell and the DISM command, follow these steps:

Open PowerShell with administrative privileges.
Enter the following command and press Enter:
```
Get-WindowsOptionalFeature -Online | Where-Object {$_.FeatureName -eq "VirtualMachinePlatform"}
```
If WSL 2 is enabled, you’ll see output similar to:
```
FeatureName : VirtualMachinePlatform
```
Additionally, you can use the DISM command to check:
```
dism.exe /online /get-features | Where-Object { $_ -match "Microsoft-Windows-Subsystem-Linux" }
```
Output:
```
Feature Name : Microsoft-Windows-Subsystem-Linux
```
Or you can simply go to ‘Turn Windows features on or off‘ in your Windows settings to verify if the Windows Subsystem for Linux is enabled.


To install Ubuntu 22
```
wsl --update
wsl --list --online 
```
Then install Ubuntu 22.04
```
wsl --install -d Ubuntu-22.04
```
Upon installation follow the instructions and create a username and password for your Ubuntu install. You can now launch Ubuntu via simply searching in start menu or from Windows Terminal.


#### Alternative (using docker and distrobox to get Ubuntu 22)

// Least recommended - Recommended if you want to try variety of distros and have expirience with docker

Ensure Docker is installed on your Ubuntu 20.04 system. If not installed, you can install it using:
(If on a debian based machine)
```
sudo apt update
sudo apt install docker.io
```

Setting up [distrobox](https://github.com/89luca89/distrobox):
```
wget -qO- https://raw.githubusercontent.com/89luca89/distrobox/main/install | sudo sh
```


```
distrobox create --image ubuntu:22.04 --name ubuntu22


distrobox enter ubuntu22
```


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

The ROS package ```ros_world``` contains all the files required for this task<br> 
This completes the environment setup for the autonmation task.
