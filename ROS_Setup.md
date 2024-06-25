# Instructions to set up ROS2 + Gazebo + Turtlebot3


Instructions to setup your environment is divided into following broad sections:

1. Setting up the environment itself (installing Ubuntu 22 or equivalent)
2. Setting up ROS2 (we recommend using Humble)
3. Setting up Turtlebot3 and its other packages



## Setting up Your environment

### Recommended

We recommend using **Ubuntu 22** - you do not need to remove Windows. You can dual boot Ubuntu 22.04 by [following these instructions](https://www.xda-developers.com/dual-boot-windows-11-linux/). You can download the Ubuntu 22 Desktop image from [here](https://releases.ubuntu.com/jammy/).

### Alternative (using Windows Subsystem for Linux)

This approach uses [Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/about) to run any Linux distro on your Windows machine itself.

Open PowerShell on your Windows machine and run
```
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

Open PowerShell with administrative privileges.
Enter the following command and press Enter:
```
Get-WindowsOptionalFeature -Online | Where-Object {$_.FeatureName -eq "VirtualMachinePlatform"}
```
If WSL 2 is enabled, you’ll see output similar to:
```
FeatureName : VirtualMachinePlatform
```
Or, you can simply go to ‘Turn Windows features on or off‘ in your Windows settings to verify if the Windows Subsystem for Linux is enabled.


To install Ubuntu 22
```
wsl --update
wsl --list --online
```
Then install Ubuntu 22.04
```
wsl --install -d Ubuntu-22.04
```
Upon installation, follow the instructions and create a username and password for your Ubuntu install. You can now launch Ubuntu via simply searching in start menu or from Windows Terminal.


### Alternative (using docker and distrobox to get Ubuntu 22)

// Least recommended - Recommended if you want to try a variety of distros and have experience with docker

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


## Setting up ROS2

> We recommend using ROS2 Humble

Steps to install ROS2 Humble on Ubuntu 22 [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Make sure to do Full Desktop install: `sudo apt install ros-humble-desktop-full`

Installing Gazebo (Not required separately if you have installed ```ros-humble-desktop-full```)

To verify, use this command:
```
gazebo --version
```
If you are able to see the version, Gazebo is installed.


It is **highly recommended** to go through all [tutorials](https://docs.ros.org/en/humble/Tutorials.html) on ROS2's website as that will give a very good grasp on what is ROS and how does it work.

## Setting up Turtlebot3

(**NOTE:** Change kinetic **to humble** if you follow the instructions in these links)

**Ensure** you go through turtlebot's normal [setup instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) before following the below. Also do build the package from source as shown in drop down on the website as it will prevent a lot of errors.

Next follow turtlebot3's [Setup manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

We will be using the `waffle_pi` model.
<br>

### Alternatively

You can run the following [script](https://gist.github.com/vimarsh244/ba9adf6ae3a298180aa85adfe15193f5) in your terminal window. This will setup ROS2, Gazebo and Turtlebot for you completely. [Instructions](https://gist.github.com/vimarsh244/ba9adf6ae3a298180aa85adfe15193f5?permalink_comment_id=5099679#gistcomment-5099679) to use the script.

## Automation task package
Clone this repository in to your machine or ros workspace.
```
cd ~/ros_ws/src/
```
```
git clone https://github.com/ERC-BPGC/ERC-hackathon-2024.git
```

The ROS package ```hackathon_automation``` contains all the files required for this task<br>
This completes the environment setup for the automation task.

Make sure you have all environment variables in your `.bashrc` and have sourced the file. `source ~/.bashrc`. You can check that it is configured properly by making sure that [these lines](https://gist.github.com/vimarsh244/ba9adf6ae3a298180aa85adfe15193f5?permalink_comment_id=5099676#gistcomment-5099676) are available.

You can create a symlink for hackathon_automation in your workspace.

First build the workspace

```colcon build```

Then Launch the Gazebo world by entering

```ros2 launch hackathon_automation robo.launch.py```

This completes the environment setup for the autonmation task.