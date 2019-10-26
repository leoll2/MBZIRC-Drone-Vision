#include "darknet_ros/YoloROSDetector.hpp"

namespace darknet_ros {


/* Service DetectObjects: receive an Image from the request,
 * run the Yolov3 object detection algorithm and return
 * as response an array of bounding boxes.
 */
/*bool detectObjects(darknet_ros::DetectObjects::Request &req, TODO remove
                   darknet_ros::DetectObjects::Response &res)
{
    // TODO: get image from request ('img' field)
    ROS_INFO("Executing Yolo!");
    // TODO: run algorithm
    // TODO: produce response ('bboxes field')
    return true;
}
*/

/* Initialize the darknet (YOLO) */
void YoloROSDetector::initDarknet()
{
    // TODO
}


/* Read the configuration from file */
void YoloROSDetector::readParameters()
{
    // TODO
}


/* Configure and advertise the DetectObjects service */
/*void initDetectionService() TODO remove
{
    det_srv_ = nh_.advertiseService("detect_objects", detectObjects);
    ROS_INFO("detect_objects service ready");
}*/

/* Run Yolo algorithm on the given image */
darknet_ros_msgs::BoundingBoxes YoloROSDetector::yoloDetect(cv_bridge::CvImagePtr cv_cam_img)
{
    int img_width, img_height;
    darknet_ros_msgs::BoundingBoxes bboxes_res;
    ros::Time begin_time, end_time;

    img_width = cv_cam_img->image.size().width;
    img_height = cv_cam_img->image.size().height;

    ROS_INFO("Running fake Yolo on %dx%d image...", img_width, img_height); // DEBUG
    begin_time = ros::Time::now();
    ROS_INFO("Yolo started at time: %d,%d", begin_time.sec, begin_time.nsec);
    ros::Duration(0.05).sleep();    // DEBUG simulate yolo detection delay
    // TODO run yolo
    end_time = ros::Time::now();
    ROS_INFO("Yolo finished at time: %d,%d", end_time.sec, end_time.nsec);
 
    //TODO assign bounding boxes
    return bboxes_res;
}


/* Initialize the DetectObjects action server */
void YoloROSDetector::initDetectionActionServer()
{
    detect_act_srv_.registerPreemptCallback(
        boost::bind(&YoloROSDetector::detectObjectsActionPreemptCallback,
        this)
    );
    detect_act_srv_.start();
}


/* DetectObjects action goal callback */
void YoloROSDetector::detectObjectsActionGoalCallback(const darknet_ros_msgs::DetectObjectsGoalConstPtr &img_act_ptr)
{
    cv_bridge::CvImagePtr cv_cam_img;
    darknet_ros_msgs::BoundingBoxes bboxes_res;
    darknet_ros_msgs::DetectObjectsResult detect_act_res;

    ROS_INFO("Inside DetectObjects action goal callback");

    // Convert the received image from ROS to OpenCV format
    sensor_msgs::Image ros_cam_img = img_act_ptr->img;

    try {
        cv_cam_img = cv_bridge::toCvCopy(ros_cam_img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& cve) {
        ROS_ERROR("cv_bridge: error converting img to OpenCV: %s", cve.what());
        return;
    }

    // Call Yolo on the received image
    yoloDetect(cv_cam_img);

    // Produce action response
    detect_act_res.bboxes = bboxes_res;
    detect_act_srv_.setSucceeded(detect_act_res);
}


/* DetectObjects action preempt callback */
void YoloROSDetector::detectObjectsActionPreemptCallback()
{
    ROS_INFO("Inside DetectObjects action preempt callback");
    // TODO
}


/* Constructor */
YoloROSDetector::YoloROSDetector(ros::NodeHandle nh)
  : nh_(nh),
    it_(nh_),
    detect_act_srv_(nh_, "detect_objects", 
        boost::bind(&YoloROSDetector::detectObjectsActionGoalCallback, 
            this, _1
        ),
        false
    )
{
    ROS_INFO("YoloROSDetector node started");

    readParameters();
    initDarknet();
    // initDetectionService(); TODO remove
    initDetectionActionServer();
}
    

/* Destructor */
YoloROSDetector::~YoloROSDetector() {}


}
