# Freefly Systems Technical Challenge

## The Challenge
Read this document: [The Challenge Question](docs/freefly_challenge_question.pdf).

## Project Description
This project is built using PX4 and uXRCE-DDS bridge to ROS2 Humble. I chose this method over directly implementing using MAVSDK-Python because I believe ROS2 provides additional considerations for safety-critical applications such as autonomous robots (such as UAVs) where the points of software failure can be compartmentalized without making breaking changes downstream on my local machine or downstream with changes to the PX4 stack itself. This project package should work indefinitely if all the build instructions are followed for the duration of the respective module's support timelines.

## Build
First, follow the instructions from the [ROS2 User Guide](http://docs.px4.io/main/en/ros/ros2_comm.html) in the official PX4 development website. Once you've built the ROS2 workspace, the rest of the installation and setup guide is entirely optional. Navigate to the `src` directory of your ROS2 workspace (eg: `ros2_ws`) and clone this repository using:
```shell
user@computer:<path/to/ros2_ws/src>$ git clone git@github.com:varundevsukhil/reimagined-octo-fiesta.git
```
Navigate back to the root of your ROS2 workspace, and execute the following command:
```shell
user@computer:<path/to/ros2_ws/>$ colcon build --packages-select freefly_challenge
```
If you see an output similar to the following in your terminal, then the build process is complete:
```shell
Starting >>> freefly_challenge
Finished <<< freefly_challenge [0.59s]          

Summary: 1 package finished [0.66s]
```
If the build process fails, contact the author with the error logs.

## Testing
For testing this project, we are going to use four applications that will have to be run in their own terminals.

1. The [**uXRCE DDS**](https://docs.px4.io/v1.14/en/middleware/uxrce_dds.html) agent. This application allows for bi-directional communications between the PX4 flight stack and the ROS2 middleware libraries (including the project in this repository). Use the following command to start the DDS agent:
```shell
user@computer:~$ MicroXRCEAgent udp4 -p 8888
```

2. The **PX4 SITL** (software-in-the-loop) application. This project uses the default x500 quadcopter model in a blank Gazebo world for simplicity. Navigate to the root directory of the PX4 stack on your local computer and use the following command to start the simulator:
```shell
user@computer:<path/to/px4-root-directory/>$ make px4_sitl gz_x500
```

3. The **QGroundControl**, or QGC, is the GCS software system commonly used with the PX4 flight control stack. It is generally a single executable `.AppImage` that can be launched as a normal application. This software can be downloaded from the [official source](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).

4. Finally, the technical challenge project (which includes a visualizer, a GUI interface, and the mission node) can be launched using a single ROS2 launch file provided in this repository using the following command:
```shell
user@computer:~$ ros2 launch freefly_challenge mission.launch.py
```

The mission software for this technical challenge has a 10 second delayed start to allow you to adjust the positions of the application windows (if you find this necessary). Sit back, and enjoy the show! The default mission is to trace the letters `FS` as a tribute to Freefly Systems.

## Docs
This repository has multiple supporting docs to demonstrate the results of this project when it was executed on the author's computer. The following is a brief summary:

1. [Video 1: screen capture of the Gazebo simulator and the mission visualizer](docs/video/Freefly_VarundevSukhil_Sim_and_Visualizer.mp4)

2. [Video 2: screen capture of the mission progression using the QGroundControl GCS](docs/video/Freefly_VarundevSukhil_QGC.mp4)

3. [The mission waypoint file (with header description)](mission/waypoints.csv) - change waypoints as needed and recompile the project

4. Flight logs:
    
    1. [The log `ulg` file](test/logs/mission_log.ulg)
    2. [Link to the PX4 logs hosted online](https://review.px4.io/plot_app?log=8a0cab21-09fd-449b-b79b-001521b58a27)
    3. [Copy of the flight review](docs/flight_review.pdf)
    4. [Copy of the PID analysis](docs/pid_analysis.pdf)

5. [System design document](docs/system_design.md)

## About Me
[Varundev Sukhil](https://vsukhil.com) is a computer engineer and a roboticist.