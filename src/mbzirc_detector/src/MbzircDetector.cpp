#include "ros/console.h"
#include "topic_tools/MuxSelect.h"
#include "mbzirc_detector/MbzircDetector.hpp"

/* Read the configuration from file */
void MbzircDetector::readParameters()
{
    nh_.getParam("subscribers/input_camera/topic", input_camera_topic);
    
    nh_.getParam("cameras/long_dist/name", long_camera_name);
    nh_.getParam("cameras/long_dist/topic", long_camera_topic);
    nh_.getParam("cameras/long_dist/stereo", long_camera_stereo);
    nh_.getParam("cameras/short_dist/name", short_camera_name);
    nh_.getParam("cameras/short_dist/topic", short_camera_topic);
    nh_.getParam("cameras/short_dist/stereo", short_camera_stereo);
}


/* Change the detector input channel, i.e. the videocamera to be read */
void MbzircDetector::switchCamera(std::string new_cam_topic)
{
    topic_tools::MuxSelect srv;
    srv.request.topic = new_cam_topic;
    if (cam_sel_client.call(srv)) {
        ROS_DEBUG("Switched camera from %s to %s", 
            srv.response.prev_topic.c_str(), new_cam_topic.c_str()
        );
    } else {
        ROS_ERROR("Failed to switch camera");
    }
}


/* Constructor */
MbzircDetector::MbzircDetector(ros::NodeHandle nh)
  : nh_(nh),
    it_(nh_),
    mux_nh_("mux_cam")
{
    readParameters();
    cam_sel_client = mux_nh_.serviceClient<topic_tools::MuxSelect>("select");
    
    image_sub_ = it_.subscribe(input_camera_topic, 1, 
        &MbzircDetector::cameraCallback, this
    );

}


/* Destructor */
MbzircDetector::~MbzircDetector() {}


/* Callback for new camera input */
void MbzircDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("Detector received a new input frame");
    // switchCamera(short_camera_topic);
}

