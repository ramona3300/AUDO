# PSES Docs

This README aims to show you a clear overview about the different packages / relevant files in the PSES Workspace and to give you quick access to their documentation.

## Essential
The following ROS-packages and files make up the core of the PSES and are essential to work with the cars at our lab. 
 * [ROS workshop](https://github.com/tud-pses/pses_docs/blob/master/PSES%20Einfu%CC%88hrung%20ROS.pdf): Slides with information and examples for ROS beginners as well as basic information about the PSES packages.
 * [Virtual Machine](https://github.com/tud-pses/pses_docs/blob/master/VirtualMachine.md): Download this image if you desire to have a virtual machine of Lubuntu 18.04 with ROS-Melodic and our development environment working out of the box.
 * [Installation guide](https://github.com/tud-pses/pses_docs/tree/master/installation-scripts): If you do not want to use a virtual machine or you had any hardware compatibility problem while trying that, then you also have the oportunity to follow this guide and install our development environment from scratch on your computer (requires Ubuntu 18.04 or Lubuntu 18.04).
 * [PSES UcBridge](https://github.com/tud-pses/pses_ucbridge/wiki) The ROS-package ucBridge is the heart of the PSES workspace because it enables you to communicate with the microcontroller of the car in an easy way through the "ROS world". Hence it is essential to read the wiki of this package, in order to understand how to get sensor values and send commands to the microcontroller.
 * [Hello World example](https://github.com/tud-pses/pses_helloworld): After reading the UcBridge wiki, we highly recommend to continue with this "hello world" example.

## Extra
The following packages and files are aimed to give you extra tools to make things easier and to solve some problems you might encounter. As a PSES participant is not expected of you to deeply understand these packages and the content of the files. Feel free to try them but please be advised that the Packages _PSES_Simulation_, _PSES_Dashboard_, _PSES_Kinect_Filter_, _PSES_Odometry_, _CarControlApp_ still in development and are part of our reasearch work. Hence bugs, erros and wrong results in some situations might occur. 
  * [PSES Simulation](https://github.com/tud-pses/pses_simulation/wiki): This ROS-package provides a simple simulation for the kinematic model, the ultrasonic range sensors and the kinect of the PSES robot from the Real Time Systems lab at FG Echtzeitsysteme - TU-Darmstadt. The driving simulation is based on the Ackermann kinematic model (Cars) and does not account for torque, friction or any forces. Sensory information is simulated by scanning for obstacles on a given 2D grid map at the position of the virtual model of the car. _This ROS-Package is **already installed** in all PSES cars and in the image for the virtual machine._ 
  * [PSES Dashboard](https://github.com/tud-pses/pses_dashboard/wiki): This ROS-package provides a simple GUI to control the PSES robot and display sensor information. _This ROS-package is **already installed** in all PSES cars and in the image for the virtual machine._  
  * [PSES Kinect Utilities](https://github.com/tud-pses/pses_kinect_utilities/): This ROS-package provides some utilities to cope with the noisy and dense data of the kinect and to perform efficiently some useful conversions such as depth image to point cloud or point cloud to laserscan.
  * [PSES Odometry](https://github.com/tud-pses/pses_odometry): This packages provides a very crude odometry, calculated from (unfiltered) imu and rotary encoder messages.
  * [CarControl App](https://github.com/tud-pses/CarControl-App/releases): Android app for remote controlling a PSES robot.
  * [UcBoard](https://github.com/tud-pses/ucboard): If you are interested in the microcontroller code, this is the repository to look into.
  * [UcBoard documentation](https://github.com/tud-pses/ucboard/blob/master/ucboard.pdf): Detailed documentation of the microcontroller and its firmware.
  * [UcBoard Schematics](https://github.com/tud-pses/ucboard/blob/master/ucboard_schematic.pdf): Circuit schematics of the microcontroller.
  * [Rosbag](https://github.com/tud-pses/pses_docs/blob/master/Rosbag.md): Rosbag with sensor and kinect data published by one of our model cars and the kinect v2 mounted on it. Useful for you, if you want to test your algorithms or any ROS package based on real data.

## C++ Code Documentation
  * [PSES UcBridge](https://tud-pses.github.io/pses_ucbridge/): This is the documentation of the C++ code of our core ROS-package UcBridge and the ROS-Services that it provides.

## Authors

* **Nicolas Acero**
* **Sebastian Ehmes** 

