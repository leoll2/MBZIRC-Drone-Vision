#include <ros/ros.h>
#include <signal.h>
#include <darknet_ros/YoloROSDetector.hpp>


int main(int argc, char** argv) {

  ros::init(argc, argv, "darknet_ros");

  ros::NodeHandle nodeHandle("~");
  darknet_ros::YoloROSDetector yoloROSDetector(nodeHandle);

  ros::spin();
  return 0;
}

