#include <algorithm>
#include "darknet_ros/YoloROSDetector.hpp"

namespace darknet_ros {

/* Initialize the darknet (YOLO) */
void YoloROSDetector::initDarknet()
{
    ROS_INFO("Initializing Yolo detector. cfg=%s weights=%s",
        yolo_cfg_path.c_str(), yolo_weights_path.c_str());
    detector = new Detector(yolo_cfg_path, yolo_weights_path);
    detector->nms = yolo_nms_thresh;    // non maximum suppression
}


/* Read the configuration from file */
void YoloROSDetector::readParameters()
{
    ROS_INFO("Reading Yolo parameters");
    
    // parameters in config/<model>.yaml
    nh_.getParam("yolo_model/detection_classes/names", yolo_class_labels);
    nh_.getParam("yolo_model/threshold/value", yolo_thresh);
    nh_.getParam("yolo_model/weight_file/name", yolo_weights_model);
    nh_.getParam("weights_path", yolo_weights_path);
    nh_.getParam("yolo_model/config_file/name", yolo_cfg_model);
    nh_.getParam("config_path", yolo_cfg_path);
    nh_.getParam("yolo_model/nms_threshold/value", yolo_nms_thresh);
    
    yolo_weights_path += "/" + yolo_weights_model;
    yolo_cfg_path += "/" + yolo_cfg_model;
    yolo_data_path = darknet_path + "/data";
}


/* Resize image with padding (letterbox) */
cv::Mat resize_pad_img(const cv::Mat &src, unsigned pad_size_h, unsigned pad_size_v)
{
    cv::Mat dst;
    cv::Scalar pad_color(0, 0, 0);
    copyMakeBorder(src, dst, pad_size_v, pad_size_v, pad_size_h, pad_size_h, 
        cv::BORDER_CONSTANT, pad_color
    );
    return dst;
}


/* Run Yolo algorithm on the given image */
darknet_ros_msgs::BoundingBoxes YoloROSDetector::yoloDetect(cv_bridge::CvImagePtr cv_cam_img)
{
    int img_width, img_height;
    int pad_size_h, pad_size_v;
    cv::Mat square_cam_img;
    std::shared_ptr<image_t> imaget_cam_img;
    std::vector<bbox_t> bboxes_vec_res;
    darknet_ros_msgs::BoundingBoxes bboxes_ros_res;
    ros::Time begin_time, end_time;

    // Resize image to make it square
    pad_size_h = pad_size_v = 0;
    if (cv_cam_img->image.cols > cv_cam_img->image.rows) {
        // horizontal image
        pad_size_v = (cv_cam_img->image.cols - cv_cam_img->image.rows)/2;
        square_cam_img = resize_pad_img(cv_cam_img->image, 0, pad_size_v);
    } else if (cv_cam_img->image.rows > cv_cam_img->image.cols) {
        // vertical image
        pad_size_h = (cv_cam_img->image.rows - cv_cam_img->image.cols)/2;
        square_cam_img = resize_pad_img(cv_cam_img->image, pad_size_h, 0);
    } else {
        // already square img
        square_cam_img = cv_cam_img->image;
    }
    img_width = square_cam_img.size().width;
    img_height = square_cam_img.size().height;

    // Convert from cv::Mat to darknet image_t
    imaget_cam_img = detector->mat_to_image_resize(square_cam_img);

    // Run Yolo
    begin_time = ros::Time::now();
    ROS_DEBUG("Yolo started at time: %d,%09d", begin_time.sec, begin_time.nsec);
    bboxes_vec_res = detector->detect_resized(*imaget_cam_img, 
        img_width, img_height, yolo_thresh, false);
    end_time = ros::Time::now();
    ROS_DEBUG("Yolo finished at time: %d,%09d", end_time.sec, end_time.nsec);
    ROS_DEBUG("Found %d objects", (int)bboxes_vec_res.size());
 
    // Convert bounding boxes to ROS format (compensating for previous padding)
    for (auto &b : bboxes_vec_res) {
        darknet_ros_msgs::BoundingBox ros_bbox;
        ros_bbox.Class = yolo_class_labels[b.obj_id];
        ros_bbox.prob = b.prob;
        ros_bbox.x = std::max(0, int(b.x)-pad_size_h);
        ros_bbox.y = std::max(0, int(b.y)-pad_size_v);
        ros_bbox.w = b.w;
        ros_bbox.h = b.h;
        bboxes_ros_res.bounding_boxes.push_back(ros_bbox);
    }
    return bboxes_ros_res;
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

    // Convert the received image from ROS to OpenCV format
    sensor_msgs::Image ros_cam_img = img_act_ptr->img;

    try {
        cv_cam_img = cv_bridge::toCvCopy(ros_cam_img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& cve) {
        ROS_ERROR("cv_bridge: error converting img to OpenCV: %s", cve.what());
        return;
    }

    // Call Yolo on the received image
    bboxes_res = yoloDetect(cv_cam_img);

    // Produce action response
    detect_act_res.bboxes = bboxes_res;
    detect_act_srv_.setSucceeded(detect_act_res);
}


/* DetectObjects action preempt callback */
void YoloROSDetector::detectObjectsActionPreemptCallback()
{
    ROS_DEBUG("Inside DetectObjects action preempt callback");
    // TODO
}


/* Constructor */
YoloROSDetector::YoloROSDetector(ros::NodeHandle nh)
  : nh_(nh),
    it_(nh_),
    detect_act_srv_(nh_, "detect_objects", 
        boost::bind(&YoloROSDetector::detectObjectsActionGoalCallback, 
            this, _1
        ), false
    )
{
    ROS_INFO("YoloROSDetector node started");

    readParameters();
    initDarknet();
    initDetectionActionServer();
}
    

/* Destructor */
YoloROSDetector::~YoloROSDetector() {}

}
