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
    mux_nh_("mux_cam"),
    yolo_act_cl_(nh_, "/darknet_ros/detect_objects", true)
{
    // Read the configuration
    readParameters();
    ROS_INFO("Successfully read parameters");

    // Init service client for camera switching
    cam_sel_client = mux_nh_.serviceClient<topic_tools::MuxSelect>("select");
    ROS_INFO("Successfully initialized service client for input selection");

    // Init action client for yolo detection
    yolo_act_cl_.waitForServer();
    ROS_INFO("Successfully initialized action client for Yolo");

    // Subscribe to camera input
    image_sub_ = it_.subscribe(input_camera_topic, 1, 
        &MbzircDetector::cameraCallback, this
    );
    ROS_INFO("Successfully subscribed to input camera topic");
}


/* Destructor */
MbzircDetector::~MbzircDetector() {}


/* Callback for new camera input */
void MbzircDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time begin_time, end_time;

    ROS_INFO("Detector received a new input frame");
    // switchCamera(short_camera_topic);

    // Request a DetectObjects action
    darknet_ros_msgs::DetectObjectsGoal det_goal;
    det_goal.img = *msg;
    begin_time = ros::Time::now();
    ROS_INFO("Sending DetectObjects goal at time: %d,%d", begin_time.sec, begin_time.nsec);
    yolo_act_cl_.sendGoal(det_goal);

    bool dl_no_miss = yolo_act_cl_.waitForResult(ros::Duration(0.5));
    if (dl_no_miss) {
        actionlib::SimpleClientGoalState state = yolo_act_cl_.getState();
        ROS_INFO("Yolo detection finished: %s", state.toString().c_str());
    } else {
        yolo_act_cl_.cancelGoal();
        ROS_WARN("Deadline miss for Yolo detection");
    }
    end_time = ros::Time::now();
    ROS_INFO("Finished DetectObjects goal at time: %d,%d", end_time.sec, end_time.nsec);
}

