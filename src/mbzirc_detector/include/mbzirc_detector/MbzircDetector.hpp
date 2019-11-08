#pragma once

#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>

#include <actionlib/client/simple_action_client.h>
#include <darknet_ros_msgs/DetectObjectsAction.h>
#include "BBox.hpp"
#include "color_detector/cv_algorithms.hpp"
#include "color_detector/ColorDetector.hpp"
#include "distance_finder/DistanceFinder.hpp"


class MbzircDetector {
    enum CameraType {SHORT_RANGE, LONG_RANGE};
    enum DetectType {YOLO, COLOR_AND_YOLO};
    
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::NodeHandle mux_nh_;
    ros::Publisher bboxes_pub_;
    ros::Publisher det_img_pub_;
    ros::ServiceClient cam_sel_client;
    actionlib::SimpleActionClient<darknet_ros_msgs::DetectObjectsAction> yolo_act_cl_;
    actionlib::SimpleActionClient<distance_finder::GetDistanceAction> dist_act_cl_;
    ColorDetector *color_detector;

    CameraType current_cam_range;
    DetectType det_strategy;
    std::string input_camera_topic;
    std::string long_camera_name;
    std::string long_camera_topic;
    bool long_camera_stereo;
    std::string short_camera_name;
    std::string short_camera_topic;
    bool short_camera_stereo;
    std::string bboxes_topic;
    bool bboxes_topic_enable;
    int bboxes_q_size;
    bool bboxes_latch;
    std::string det_img_topic;
    bool det_img_topic_enable;
    int det_img_q_size;
    bool det_img_latch;

    void readParameters();
    void initColorDetector();
    void switchCamera(CameraType new_camera_range);
    std::vector<BBox> callYoloDetector(const sensor_msgs::ImageConstPtr& msg_img);
    std::vector<BBox> callYoloDetector(const cv::Mat &img, std_msgs::Header header);
    std::vector<BBox> callColorDetector(const cv::Mat &img);
    std::vector<BBox> detect(const sensor_msgs::ImageConstPtr& msg_img);
public:
    MbzircDetector(ros::NodeHandle nh);
    ~MbzircDetector();
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
};

