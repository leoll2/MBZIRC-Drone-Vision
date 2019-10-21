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
void YoloROSDetector::detectObjectsActionGoalCallback()
{
    // TODO
    ROS_INFO("Inside DetectObjects action goal callback");
}


/* DetectObjects action preempt callback */
void YoloROSDetector::detectObjectsActionPreemptCallback()
{
    // TODO
    ROS_INFO("Inside DetectObjects action preempt callback");
}


/* Constructor */
YoloROSDetector::YoloROSDetector(ros::NodeHandle nh)
  : nh_(nh),
    it_(nh_),
    detect_act_srv_(nh_, "detect_objects", 
        boost::bind(&YoloROSDetector::detectObjectsActionGoalCallback, 
            this
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
