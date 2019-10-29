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


/* Invoke detection with Yolo algorithm */
std::vector<BBox> MbzircDetector::callYoloDetector(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time begin_time, end_time;
    float deadline = 0.5;
    darknet_ros_msgs::DetectObjectsGoal det_goal;
    darknet_ros_msgs::DetectObjectsResultConstPtr det_res_ptr; 
    std::vector<BBox> res_bboxes;

    det_goal.img = *msg;
    begin_time = ros::Time::now();
    ROS_DEBUG("Sending DetectObjects goal at time: %d,%d", begin_time.sec, begin_time.nsec);
    yolo_act_cl_.sendGoal(det_goal);

    bool dl_no_miss = yolo_act_cl_.waitForResult(ros::Duration(deadline));
    if (dl_no_miss) {   // deadline respected
        actionlib::SimpleClientGoalState state = yolo_act_cl_.getState();
        ROS_DEBUG("Yolo DetectObjects final state: %s", state.toString().c_str());
        det_res_ptr = yolo_act_cl_.getResult();
        for (const auto &b : det_res_ptr->bboxes.bounding_boxes) {
            BBox bbox;
            bbox.x = b.x;
            bbox.y = b.y;
            bbox.w = b.w;
            bbox.h = b.h;
            bbox.prob = b.prob;
            bbox.obj_class = b.Class;
            res_bboxes.push_back(bbox);
        }
    } else {        // deadline missed
        yolo_act_cl_.cancelGoal();
        ROS_WARN("Deadline miss for Yolo detection");
    }
    end_time = ros::Time::now();
    ROS_DEBUG("Completed DetectObjects goal at time: %d,%d", end_time.sec, end_time.nsec);

    return res_bboxes;
}


/* Callback for new camera input */
void MbzircDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::vector<BBox> bboxes;

    ROS_INFO("Detector received a new input frame");
    // Change camera
    // switchCamera(short_camera_topic);
    
    // Yolo detector
    bboxes = callYoloDetector(msg);

    // List bboxes (debug)
    ROS_INFO("Detected %d objects:", (int)bboxes.size());
    for (const auto &b : bboxes) {
        ROS_INFO("\t %s at [(%d,%d),(%d,%d)] with prob %f",
            b.obj_class.c_str(), b.x, b.y, b.x+b.w, b.y+b.h, b.prob    
        );
    }
}

