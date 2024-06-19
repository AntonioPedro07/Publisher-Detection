# Publisher-Detection

This project was developed by António Pedro Silva Matos using ROS (Robot Operating System) and OpenCV to detect objects in images captured by a camera, publishing the results in a ROS topic. The project consists of two main nodes: a node for detecting objects and another for publishing the results.

## Summary

1. [**Introduction**](#introduction)
2. [**Overview**](#overview)
3. [**Functionalities**](#functionalities)
4. [**Configuration**](#configuration)
5. [**Requirements**](#requirements)
6. [**Preperation**](#preperation)
   - [**Installation**](#installation)
   - [**ROS Installation**](#installation-ros)
7. [**Usage**](#usage)
   - [**Start detection node**](#start-detection-node)
   - [**View Topics**](#view-topics)
8. [**Contribution**](#contribution)
9. [**License**](#license)

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
