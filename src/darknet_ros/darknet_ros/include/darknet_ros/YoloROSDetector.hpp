#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

#include <actionlib/server/simple_action_server.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/DetectObjectsAction.h>

#include "yolo_v2_class.hpp"

namespace darknet_ros {

    class YoloROSDetector {
        ros::NodeHandle nh_;
        //ros::ServiceServer det_srv_;  TODO remove
        image_transport::ImageTransport it_;
        actionlib::SimpleActionServer<darknet_ros_msgs::DetectObjectsAction> detect_act_srv_;
        Detector *detector;

        //bool detectObjects(darknet_ros::DetectObjects::Request &req, TODO remove
        //                   darknet_ros::DetectObjects::Response &res);
        void initDarknet();
        void readParameters();
        darknet_ros_msgs::BoundingBoxes yoloDetect(cv_bridge::CvImagePtr cv_cam_img);
        void initDetectionActionServer();
        void detectObjectsActionGoalCallback(const darknet_ros_msgs::DetectObjectsGoalConstPtr &img_act_ptr);
        void detectObjectsActionPreemptCallback();
        //void initDetectionService(); TODO remove
    public:
        YoloROSDetector(ros::NodeHandle nh);
        ~YoloROSDetector();
    };
};
