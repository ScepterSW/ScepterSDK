
# ROS2 Wrapper for ScepterSDK

## Overview
This ROS2 package facilitates depth IR and color data acquisition and processing for ScepterSDK.

## Installation

- **Install the recommended [ROS distribution](http://wiki.ros.org/Distributions) for your operating system**
  - [ROS2 Install page](http://docs.ros.org/en/rolling/Installation.html)

  **Version verified**
  |system|details|
  |---|---|
  |Ubuntu20.04|Foxy Fitzroy|
  |Ubuntu18.04|Eloquent Elusor|

- **Install the Scepter ROS2 package**

  - [Install ScepterSDK](https://github.com/Scepter/ScepterSDK)
    
    ```console
    git clone https://github.com/Scepter/ScepterSDK
    ```
    <p align="center"><img src="./doc/img/step0.png" /></p>
  - **Update SDK to ROS2 package**
    
    ```console
    cd ROS2/src/ScepterROS_MultiCameras
    ```
  <p align="center"><img src="./doc/img/step2.png" /></p>
  
    <b>install.py</b>: copy <b>ScepterSDK</b> (match with your operating system) to <b>dependencies</b>, with the cmd "<b>python install.py (your operating system)</b>", take <b>Ubuntu18.04</b> as an example：
    ```console
    python install.py Ubuntu18.04
    ```
    <p align="center"><img src="./doc/img/step3.png" /></p>

 - **Build the ScepterROS2 package**

  If not installed colcon, run the cmd first:
  ```console
  sudo apt install python3-colcon-common-extensions
  ```
  ```console
  cd ../../
  colcon build --packages-select ScepterROS_MultiCameras
  ```
  <p align="center"><img src="./doc/img/step4.png" /></p>


 - **Environment setup**
  ```console
  source install/setup.bash 
  ```
## Usage
- **Starting camera node**
  
    ```console
    ros2 run ScepterROS_MultiCameras scepter_multicameras <nodename> <ip>
    ```
    such as:
    ```console
    ros2 run ScepterROS_MultiCameras scepter_multicameras cam1 192.168.1.102
    ```
    <p align="center"><img src="./doc/img/step6.png" /></p>
    
- **Rviz**
  
    Only one topic can be displayed at a time

    - <b>With Rviz show frame</b>
    
    ```console
    ros2 run rviz2 rviz2
    ```
    <p align="center"><img src="./doc/img/step7.png" /></p>
    
    <p align="center"><img src="./doc/img/step8.png" /></p>
    
    - <b>With Rviz show PointCloud</b>
    <p align="center"><img src="./doc/img/step10.png" /></p>
    
    <p align="center"><img src="./doc/img/step11.png" /></p>

## Published Topics
The scepter_manager publishes messages defined by the [sensor_msgs](http://wiki.ROS2.org/sensor_msgs) package on the following topics

- /<b>nodename</b>/color/camera_info
- /<b>nodename</b>/color/image_raw
- /<b>nodename</b>/depth/camera_info
- /<b>nodename</b>/depth/image_raw
- /<b>nodename</b>/depth/points
- /<b>nodename</b>/depth/points/camera_info
- /<b>nodename</b>/ir/camera_info
- /<b>nodename</b>/ir/image_raw
- /<b>nodename</b>/transformedColor/camera_info
- /<b>nodename</b>/transformedColor/image_raw
- /<b>nodename</b>/transformedDepth/camera_info
- /<b>nodename</b>/transformedDepth/image_raw
  
    <p align="center"><img src="./doc/img/topic.png" /></p>
## Programming guide
If developers need to set camera parameters or algorithm switches, please refer to the following process.
Take calling <b>scSetSpatialFilterEnabled</b> as an example
- Find the api From **dependencies/Include/Scepter_api.h**
<p align="center"><img src="./doc/img/step13.png" /></p>

- Add the code into **/src/scepter_manager.cpp**
<p align="center"><img src="./doc/img/step14.png" /></p>

## Details
- When using multiple network cards, set different IP network segments