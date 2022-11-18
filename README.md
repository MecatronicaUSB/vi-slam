# Visual Inertial SLAM

Implementation of Monocular Simultaneous Localization and Mapping (SLAM) using inertial sensors. This implementation used OpenCV 3.2 and ROS Kinect.

VI-SLAM is a free and open source licensed under the [GPL-3.0 License](https://en.wikipedia.org/wiki/GNU_General_Public_License).

## Table of Contents
- [Requirements](#requirements)
- [Getting Started](#getting-started)
- [Directory Layout](#directory-layout)
- [License](#license)

## Requirements

- [OpenCV 3.2](http://opencv.org) and extra modules.
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation).

## Getting Started

From a fresh Ubuntu 16.04 LTS, install the following dependencies:

#### OpenCV 3.2 

Refer to [OpenCV](https://github.com/MecatronicaUSB/uwimageproc/blob/master/INSTALL.md).

#### ROS Kinetic

Refer to [ROS Kinetic installation instructions](http://wiki.ros.org/kinetic/Installation).

## Building VI-SLAM

Clone VI-SLAM repository in the `/src` folder of your catkin workspace:

```bash
cd <catkin_ws_directory>/src
git clone https://github.com/MecatronicaUSB/vi-slam.git
cd ..
catkin_make
```

## Usage

### General datasets

Run VI-SLAM on a dataset of images with known camera calibration parameters and image dimensions and sampling rate of camera and IMU.

Modify the `calibration.xml` file in `/calibration` folder to specify the instrinsic parameters of the camera of the dataset to use. 

```bash
    -d <directory of images files>                  
    -c <directory of calibration.xml file>          (<vi-slam directory>/calibration/calibration.xml)
    -s <number of starting frame>                   (Default: 0)
```

Modify the `vi_slam.launch` file in `/launch` folder to specify the directory of files  
(Refer to `/calibration/calibrationEUROC.xml` for proper configuration of the .xml file).

```bash
    <!-- Images dimensions (Input) -->
    <in_width  type_id="integer"> W </in_width>       (Input dimentions of images)
    <in_height type_id="integer"> H </in_height>      (Replace W: width, H: height)

    <!-- Images dimensions (Output) -->
    <out_width  type_id="integer"> W </out_width>     (Output desired dimentions of images)
    <out_height type_id="integer"> H </out_height>    (Replace W: width, H: height)

    <!-- Calibration Values of Dataset -->
    <calibration_values type_id="opencv-matrix">      (Intrinsic camera parameters)
    <rows>1</rows>                                    (Replace fx fy cx cy)
    <cols>4</cols>
    <dt>f</dt>
    <data>
        fx  fy  cx  cy </data></calibration_values> 

    <!-- Distortion coefficients -->
    <rectification type_id="opencv-matrix">          (Distortion parameters, optional)
    <rows>1</rows>                                   (Replace k1 k2 k3 k4)
    <cols>4</cols>                                   (If not: 0  0  0  1)
    <dt>f</dt>  
    <data>
        k1  k2  k3  k4 </data></rectification>
```

Run VI-SLAM for general datasets:
```bash
roslaunch vi_slam vi_slam.launch
```
### EUROC dataset

Currently, VI-SLAM supports ground-truth visualization along with VI-SLAM results for [EUROC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) datasets. For these datasets, corresponding `calbration.xml` files are located in the `/calibration` folder.

#### EUROC

For [EUROC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) datasets, modify the arguments of the `vi_slamEUROC.launch` file in `/launch` folder to add the directory of the files.
```bash
    -d <directory of images files>                  (<EUROC directory>/mav0/cam0/data/)
    -c <directory of calibrationEUROC.xml file>     (<vi-slam directory>/calibration/calibrationEUROC.xml)
    -s <number of starting frame>                   (Default: 0)
    --EUROC <directory of ground-truth poses file>  (<EUROC directory>/mav0/vic0/data.csv)
```
Run VI-SLAM for EUROC datasets:
```bash
roslaunch vi_slamvi_slamEUROC.launch
```


## Software Details

- Implementation done in C++.
- Using Rviz as visualization tool.

## Directory Layout

#### /src

Core .cpp files of VI-SLAM.

#### /include

Libraries .h files of VI-SLAM.  

#### /launch

VI-SLAM launch files for easy ROS Kinetic execution.

#### /calibration

Calibration files. Includes calibration for EUROC  datasets. It can be modified for local datasets.

## License

Copyright (c) 2019 Luis Lujano (<lugaluchinchilla@gmail.com>).

Release under the [GPL-3.0 License](LICENSE). 
