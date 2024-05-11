
# ROS Wrapper for ScepterSDK

## Overview
This ROS package facilitates depth IR and color data acquisition and processing for ScepterSDK.

## Installation

- **Install the recommended [ROS distribution](http://wiki.ros.org/Distributions) for your operating system**
  - [ROS Install page](http://wiki.ros.org/ROS/Installation)

  **Version verified**
  |system|details|
  |---|---|
  |Ubuntu20.04|Noetic Ninjemys|
  |Ubuntu18.04|Melodic Morenia|
  |Ubuntu16.04|Kinetic Kame|
  |AArch64|Melodic|
- **Install the Scepter ROS package**

  - [Install ScepterSDK](https://github.com/Scepter/ScepterSDK)
    
    ```console
    git clone https://github.com/Scepter/ScepterSDK
    ```
    <p align="center"><img src="./doc/img/step0.png" /></p>
  - **Update SDK to ROS package**
    
    ```console
    cd ROS/src
    catkin_init_workspace
    ```
    After run <b>catkin_init_workspace</b>, it will generate the <b>CmakeLists.txt</b> in the <b>ROS/src</b> folder
  <p align="center"><img src="./doc/img/step1.png" /></p>
    
    ```console
    cd ScepterROS
    ```
  <p align="center"><img src="./doc/img/step2.png" /></p>
    
    <b>install.py</b>: copy <b>ScepterSDK</b> (match with your operating system) to <b>dependencies</b>, with the cmd "<b>python install.py (your operating system)</b>", take <b>Ubuntu18.04</b> as an example：
    
    ```console
    python install.py Ubuntu18.04
    ```
    <p align="center"><img src="./doc/img/step3.png" /></p>

 - **Build the ScepterROS package**
  ```console
  cd ../../
  catkin_make -DCATKIN_WHITELIST_PACKAGES="ScepterROS"  
  ```
  <p align="center"><img src="./doc/img/step4.png" /></p>
  <p align="center"><img src="./doc/img/step5.png" /></p>

 - **Environment setup**
  ```console
  source devel/setup.bash 
  ```
## Usage
- **Starting camera node**
    ```console
    roslaunch ScepterROS scepter_camera.launch
    ```
    <p align="center"><img src="./doc/img/step6.png" /></p>
    - <b>With Rviz show frame</b>
    
    ```console
    rviz
    ```
    <p align="center"><img src="./doc/img/step7.png" /></p>
    <p align="center"><img src="./doc/img/step8.png" /></p>
    - <b>With RQT dynamic reconfigure</b>

    ```console
    rosrun rqt_reconfigure rqt_reconfigure
    ```
    <p align="center"><img src="./doc/img/step9.png" /></p>
    
    >**Instructions:**
    >
    >- Modifying the **FrameRate** will affect the maximum of **ToFExposureTime** and **ColorExposureTime**
    >- The value is invalid when **ToFExposureTime** or **ColorExposureTime**  is set above the maximum value
    >- **HDRMode** takes effect only when **ToFManual** is True
    >- **ToFManual** set to false is invalid when **HDRMode** is True
    
- **Show PointCloud**
  
    ```console
    roslaunch ScepterROS scepter_pointCloudxyz.launch
    ```
    <p align="center"><img src="./doc/img/step10.png" /></p>
    <p align="center"><img src="./doc/img/step11.png" /></p>
    
- **Show PointCloud with color**

    ```console
    roslaunch ScepterROS scepter_pointCloudxyzcolor.launch
    ```
    <p align="center"><img src="./doc/img/step12.png" /></p>

## Published Topics
The scepter_manager publishes messages defined by the [sensor_msgs](http://wiki.ros.org/sensor_msgs) package on the following topics
- /Scepter/depth/camera_info
- /Scepter/color/image_raw
- /Scepter/depth/image_raw
- /Scepter/ir/image_raw
- /Scepter/transformedDepth/image_raw
- /Scepter/transformedColor/image_raw

## Programming guide
If developers need to set camera parameters or algorithm switches, please refer to the following process.
Take calling <b>scSetSpatialFilterEnabled</b> as an example

- Find the api From **dependencies/Include/Scepter_api.h**
<p align="center"><img src="./doc/img/step13.png" /></p>

- Add the code into **/src/scepter_manager.cpp**
<p align="center"><img src="./doc/img/step14.png" /></p>

## Details
- When using multiple network cards, set different IP network segments