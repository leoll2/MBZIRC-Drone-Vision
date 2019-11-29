# MBZIRC Vision Module

Target detection module of the Biorobotics Team for MBZIRC 2020

## Dependencies 

[TODO]

## Install

Clone the repo:
```
git clone https://github.com/leoll2/mbzirc_detection_ws
git submodule init
git submodule update
```

Download the Yolo weights from the following links:
```
https://drive.google.com/drive/folders/1Sa31sZs0eNFPUbJJfuMr3kzoQXbMGRf1?usp=sharing
```
and put them in:
```
src/darknet_ros/darknet_ros/yolo_network_config/weights
```


## Configure

### Optimization (optional)
It is possible to optimize the darknet with architecture-specific options.
The following setup works for Nvidia Jetson TX2, but feel free to adapt it to your hardware (open the Makefile to see all options).
```
sed -i 's/GPU=0/GPU=1/g' src/darknet_ros/darknet/Makefile
sed -i 's/CUDNN=0/CUDDN=1/g' src/darknet_ros/darknet/Makefile
sed -i 's/CUDNN_HALF=0/CUDNN_HALF=1/g' src/darknet_ros/darknet/Makefile
sed -i 's/OPENCV=0/OPENCV=1/g' src/darknet_ros/darknet/Makefile
sed -i 's/OPENMP=0/OPENMP=1/g' src/darknet_ros/darknet/Makefile
sed -i 's/'# ARCH= -gencode arch=compute_62'/'ARCH= -gencode arch=compute_62'/g' src/darknet_ros/darknet/Makefile
```

### Specify your setup

First, you need to declare if you are using a stereo camera (ZED) or not.
```
catkin profile set <profile-name>
```
where `profile-name` is one of `stereo` or `no_stereo`



## Launch

Compile with:
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Run with:
```
roslaunch mbzirc_detection_launcher start_all.launch
```

## Author

- Leonardo Lai
