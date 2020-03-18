# MBZIRC Vision Module

Here is the code of the **target detection** module for **autonomous drones**, developed by me as a member of the Biorobotics Team (Sant'Anna School, Pisa) in MBZIRC 2020.  

[**MBZIRC**](https://www.mbzirc.com/) (*Mohamed Bin Zayed International Robotics Challenge)* is one of the largest and most prestigious robotics competititions in the world, held in Abu Dhabi every 2-3 years.  
In the 2020 edition, one of the challenges was to develop and build a fully autonomous drone able to identify a target (balloon) and pop it.

My team secured the 4th place in this challenge, out of 22 selected teams.

<p float="left">
<img src="http://i3.ytimg.com/vi/tmsHTf0RaPw/maxresdefault.jpg" height="300">
<img src="https://imgur.com/R3SF4hR.jpg" height="300">
</p>

**Watch the video:** https://www.youtube.com/watch?v=tmsHTf0RaPw

## Description

Our object detection module implements two strategies:
* **Computer Vision**: identify the target by color (configurable) and shape (round)
* **Deep Learning**: CNN (tiny-YOLOv3) trained on a self-made dataset with more than 5000 balloon images.

All the computations are performed on a Nvidia Jetson TX2 installed on the drone.  
The neural network has been trained with the help of a Tesla P100 GPU.

---

## Dependencies

- CUDA 10
- OpenCV 4
- ROS Melodic
- ROS packages:
  - python-catkin-tools: `sudo apt install python-catkin-tools`
- ZED SDK (if using ZED stereo camera)

## Install

Clone the repo:
```
git clone https://github.com/leoll2/mbzirc_detection_ws
git submodule init
git submodule update
```

Download the cfg and weights from the following link:
```
https://drive.google.com/drive/folders/1ml-huTmX6u92elMDoegNejKB1D4wr0xd?usp=sharing
```
Put the .cfg file in:
```
src/darknet_ros/darknet_ros/yolo_network_config/cfg
```

and the .weights file in:
```
src/darknet_ros/darknet_ros/yolo_network_config/weights
```


## Configuration

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

### Specify your camera layout

First, you need to declare if you are using a stereo camera (ZED) or not, choosing the appropriate catkin profile:
```
catkin profile set <profile-name>
```
where `profile-name` is one of `stereo` or `no_stereo`.

Then, in `src/mbzirc_detection_launcher/start_all.launch` you can specify how many cameras you are using (`single` or `dual`) and their names. You can find the list of currently supported cameras in `src/mbzirc_detector/config/cameras.yaml`, but of course you can extend it.
The layout configuration looks like this:
```
<arg name="cam_layout" default="single" />
<arg name="short_cam" default="zed" />
<arg name="long_cam" default="zed" />
```
*Note: when using the 'single' layout, specify the same camera name both in short_cam and long_cam to avoid problems.*

### Configure the remote streaming

It is possible to visualize what the drone is detecting even from a remote host.
To do so, of course, the drone must be visible in the network: you can specify the connection parameters like this:
```
<arg name="stream_port" default="8080" />
<arg name="stream_addr" default="192.168.0.21" />
<arg name="stream_type" default="h264" />
```
Make sure to correctly set your IP address (use `ip addr show`), and choose a free port.  

The receiving host can view the stream by the means of a conventional browser (e.g. Chrome/Firefox), at an address that will be very similar to the following:  
`http://192.168.0.21:8080/stream?topic=/mbzirc_detector/detection_image&type=mjpeg&quality=70`

### Configure the camera

Each camera is different from the others, and comes with a set of built-in or tunable parameters that, at last, will determine its performance. Knowing such parameters is especially important in the distance estimation stage, and that's why you can configure them in `distance_finder/config/cameras.yaml`.  
You have to specify the resolution, focal length, sensor width and if it supports stereoscopical vision.

### Configure the target parameters

It is also important to provide information about the targets to detect. In this case, all you need is basically size and color. You do this in `src/distance_finder/config/targets.yaml` and `src/mbzirc_detector/config/detection.yaml`.

### Configure the detection algorithm

In `src/mbzirc_detector/config/detection.yaml` you can also customize the behaviour of the detection algorithm.
You can either decide to rely on computer vision techniques, deep learning or both! But you can also choose what to detect and enable practical optimizations to improve the reliability.

### Other parameters

You can find different configuration files scattered across the packages. Most of the parameters allow you to change the topic names and attributes, but there are dozens for other secondary functionalities, to suit everyone's need.
Often you can find them documented in the docs page of the respective ROS package.

## Launch

Compile with:
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

Run with:
```
roslaunch mbzirc_detection_launcher start_all.launch
```

## Record

It is possible to save the video streaming on the remote host, simply using any program that can record media streams from the network, e.g. the popular VLC media player.
In VLC, under "Media", choose "Convert/Save". Select "Network", then insert the streaming URL (see above), and finally click the convert button below. As profile, I suggest using H264 + MP3 (MP4). Choose a folder and a name for the file you're going to create. The recording will start as soon as you click on the button "Start", and you will see a red marker in the bottom (you're recording, even though it is not that explicit). To finish recording, click the stop button (i.e. the square).
Be aware that the very last moments/seconds of the video may go missing, if they were in cache waiting to be written when you shut the system down.

## Performances
- The color-based detection algorithm runs smoothly at more than 20 FPS on a Jetson TX2. Its high-speed makes it ideal to identify small targets moving fast.
- The deep learning detection algorithm (YOLOv3) runs at 6 FPS. Although slower, it is more robust against false positives, and does fine even in hard conditions (e.g. white balloon in white background). It works best to detect static targets.

### Troubleshooting

If the build process fails, it is likely that some ROS package is missing. Please check the error message looking for the needed package, and possibly open an issue so that I can add it as dependency.

ROS offers several tools to monitor the state of your nodes and topics. In case of problems, they become your best friend.
I strongly suggest:
- `rqt_graph`: display the ROS system architecture as a graph, making it immediate to see if everything is plugged correctly
- `rqt_console`: interface to display and filter log messages. No need to say more.
- `rostopic echo <topic_name>`: print what is being written on the topic. Useful to see if you're sending the right information, or if nothing is being published at all!
- `rostopic hz <topic_name>`: tells you how often the topic is being published. Great for performance analysis.

## Limitations

- The color-based algorithm currently supports only the detection of the primary target. Indeed, it is a hard task to detect secondary targets (white) by color with a decent accuracy, since a large fraction of the background environment is white as well.

## Report a bug

Feel free to report any issue you experience with this package, or make a pull request if you have something to fix/extend.

## Author

- Leonardo Lai
