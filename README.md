# Publisher-Detection

This project was developed by António Pedro Silva Matos using ROS (Robot Operating System) and OpenCV to detect objects in images captured by a camera, publishing the results in a ROS topic. The project consists of two main nodes: a node for detecting objects and another for publishing the results.

## Summary

1. [**Introduction**](#introduction)
2. [**Overview**](#overview)
3. [**Functionalities**](#functionalities)
4. [**Requirements**](#requirements)
5. [**Preperation**](#preperation)
   - [**Installation**](#installation)
   - [**ROS Installation**](#ros-installation)
6. [**Usage**](#usage)
   - [**Start Detection Node**](#start-detection-node)
   - [**View Topics**](#view-topics)
7. [**Contribution**](#contribution)
8. [**License**](#license)

## Introduction

This repository presents a platform for object detection using ROS and OpenCV. Developed to provide an efficient and robust solution, the project aims to help researchers and developers implement object detection systems that can be easily integrated into robotics applications.

## Overview

Object detection is a crucial area of ​​computer vision and robotics, with applications in navigation, manipulation and interaction with the environment. In this repository you will find:

   - Implementations of object detection algorithms using OpenCV.
     
   - Integration with ROS to publish detection results to ROS topics.
     
   - Scripts for training and evaluating object detection models.
     
   - Guides to help with system configuration and use.

## Functionalities

- Object Detection with OpenCV: Using computer vision algorithms to detect objects in images and videos.
  
- ROS integration: Publishing detection results to ROS topics to facilitate communication with other nodes in the system.
  
- Performance Comparison: Scripts to evaluate and compare the performance of detection models.
  
- Support for Diverse Data Sets: Ability to use different data sets to train and evaluate models.

## Requirements

- Python 3.7 or superior

- OpenCV 4.x

- ROS Noetic (or other version compatible)

# Preperation

### Installation

1. Clone the repository:
   ```
      git clone https://github.com/AntonioPedro07/Publisher-Detection.git
   ```

2. Navigate to the project directory:
   ```
      cd Publisher-Detection
   ```

3. Install the dependencies:
   ```
      pip install -r requirements.txt
   ```

### Ros Installation

1. Configure ROS Source:
   ```
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   ```

2. Install ROS Noetic:
   ```
   sudo apt install ros-noetic-desktop-full
   ```

3. Inicializar rosdep:
   ```
   sudo rosdep init
   rosdep update
   ```

4. Configure ROS Environment:
   Add the following lines to your ~/.bashrc file
   ```
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

5. Install dependencies to build ROS packages:
   ```
   sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

6. Create and configure a catkin workspace:
   ```
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```
   Add the following line to your ~/.bashrc file:
   ```
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

# Usage

### Start detection Node

To start the detection node and publish the results to ROS topics, run:
```
roslaunch publisher_detection detection.launch
```

### View Topics

To view the data published in the /distance, /almost_hit, and /warning topics, use the following commands:

1. /distance: This topic publishes the distance between the robot and detected objects
   ```
   rostopic echo /distance
   ```

2. /almost_hit: This topic publishes warnings when the robot is about to collide with an object.
   ```
   rostopic echo /almost_hit
   ```

3. /warning: This topic publishes general warnings related to object detection
   ```
   rostopic echo /warning
   ```

## Contribution

Feel free to open issues and submit pull requests. Contributions are welcome!

## License

Publisher-Detection is licensed under the MIT License - see the [LICENSE](https://github.com/AntonioPedro07/YoloKeras-Detection/blob/main/LICENSE) file for more details.
