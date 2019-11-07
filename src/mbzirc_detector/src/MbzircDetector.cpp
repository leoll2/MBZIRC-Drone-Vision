#include "ros/console.h"
#include "topic_tools/MuxSelect.h"
#include "mbzirc_detector/MbzircDetector.hpp"

/* Read the configuration from file */
void MbzircDetector::readParameters()
{
    nh_.getParam("subscribers/input_camera/topic", input_camera_topic);

    nh_.getParam("publishers/bounding_boxes/topic", bboxes_topic);
    nh_.getParam("publishers/bounding_boxes/enable", bboxes_topic_enable);
    nh_.getParam("publishers/bounding_boxes/queue_size", bboxes_q_size);
    nh_.getParam("publishers/bounding_boxes/latch", bboxes_latch);
    
    nh_.getParam("publishers/detection_image/topic", det_img_topic);
    nh_.getParam("publishers/detection_image/enable", det_img_topic_enable);
    nh_.getParam("publishers/detection_image/queue_size", det_img_q_size);
    nh_.getParam("publishers/detection_image/latch", det_img_latch);

    nh_.getParam("cameras/long_dist/name", long_camera_name);
    nh_.getParam("cameras/long_dist/topic", long_camera_topic);
    nh_.getParam("cameras/long_dist/stereo", long_camera_stereo);
    nh_.getParam("cameras/short_dist/name", short_camera_name);
    nh_.getParam("cameras/short_dist/topic", short_camera_topic);
    nh_.getParam("cameras/short_dist/stereo", short_camera_stereo);
}


/* Initialize the color detector, according to its config file */
void MbzircDetector::initColorDetector()
{
    int hue_min, hue_max, sat_min, sat_max, val_min, val_max;
    int min_area_pix;
    int max_objects;

    nh_.getParam("color_detector/hsv_thresh/hue_min", hue_min);
    nh_.getParam("color_detector/hsv_thresh/hue_max", hue_max);
    nh_.getParam("color_detector/hsv_thresh/sat_min", sat_min);
    nh_.getParam("color_detector/hsv_thresh/sat_max", sat_max);
    nh_.getParam("color_detector/hsv_thresh/val_min", val_min);
    nh_.getParam("color_detector/hsv_thresh/val_max", val_max);

    nh_.getParam("color_detector/min_area_pix", min_area_pix);
    nh_.getParam("color_detector/max_objects", max_objects);

    std::vector<unsigned> th {(unsigned)hue_min, (unsigned)hue_max, 
        (unsigned)sat_min, (unsigned)sat_max, (unsigned)val_min, (unsigned)val_max
    };
    color_detector = new ColorDetector(th, (unsigned)min_area_pix, (unsigned)max_objects);
    ROS_INFO("Initialized color detector with thresholds=(%d,%d,%d,%d,%d,%d)\
        min_area_pix=%d max_objects=%d", hue_min, hue_max, sat_min, sat_max,\
        val_min, val_max, min_area_pix, max_objects    
    );
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
    yolo_act_cl_(nh_, "/darknet_ros/detect_objects", true),
    dist_act_cl_(nh_, "/distance_finder/get_distance", true),
    det_strategy(YOLO)
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

    // Init color detector
    initColorDetector();
    ROS_INFO("Successfully initialized color detector");

    // Publish bounding boxes and detection image
    if (bboxes_topic_enable) {
        /* TODO old code
        bboxes_pub_ = nh_.advertise<darknet_ros_msgs::BoundingBoxes>( 
            bboxes_topic, bboxes_q_size, bboxes_latch
        );
        */
        bboxes_pub_ = nh_.advertise<distance_finder::ObjectBoxes>( 
            bboxes_topic, bboxes_q_size, bboxes_latch
        );
    }
    if (det_img_topic_enable) {
        det_img_pub_ = nh_.advertise<sensor_msgs::Image>(
            det_img_topic, det_img_q_size, det_img_latch
        );
    }

    // Subscribe to camera input
    image_sub_ = it_.subscribe(input_camera_topic, 1, 
        &MbzircDetector::cameraCallback, this
    );
    ROS_INFO("Successfully subscribed to input camera topic");
}


/* Destructor */
MbzircDetector::~MbzircDetector() 
{
    delete color_detector;
}


/* Invoke detection with Yolo algorithm */
std::vector<BBox> MbzircDetector::callYoloDetector(const sensor_msgs::ImageConstPtr& msg_img)
{
    ros::Time begin_time, end_time;
    float deadline = 0.5;
    darknet_ros_msgs::DetectObjectsGoal det_goal;
    darknet_ros_msgs::DetectObjectsResultConstPtr det_res_ptr; 
    std::vector<BBox> res_bboxes;

    det_goal.img = *msg_img;
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


/* Invoke detection with Yolo algorithm */
std::vector<BBox> MbzircDetector::callYoloDetector(const cv::Mat &img, std_msgs::Header header)
{
    sensor_msgs::ImageConstPtr msg_ptr;
    cv_bridge::CvImage msg_img;

    msg_img.header = header;
    msg_img.encoding = sensor_msgs::image_encodings::BGR8;
    msg_img.image = img;

    msg_ptr = msg_img.toImageMsg();
    return callYoloDetector(msg_ptr);
}


/* Invoke detection with color-based algorithm */
inline std::vector<BBox> MbzircDetector::callColorDetector(const cv::Mat& img)
{
    return color_detector->detect(img);
}


/* Detect objects */
std::vector<BBox> MbzircDetector::detect(const sensor_msgs::ImageConstPtr& msg_img)
{
    std::vector<BBox> bboxes;
    std::vector<BBox> color_bboxes;
    std::vector<BBox> yolo_bboxes;
    cv_bridge::CvImagePtr cv_img_ptr;
    cv::Mat cv_img;
    std_msgs::Header hdr;

    switch (det_strategy) {

        case YOLO:
            // Just call Yolo
            bboxes = callYoloDetector(msg_img);
            break;

        case COLOR_AND_YOLO:
            // Grab header (to recycle later)
            hdr = msg_img->header;
            // Convert to cv::Mat
            try {
                cv_img_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& cve) {
                ROS_ERROR("cv_bridge: error converting img to OpenCV: %s", cve.what());
                break;
            }
            cv_img = cv_img_ptr->image;
            // Perform detection by color
            color_bboxes = callColorDetector(cv_img);
            // For each ROI, perform a local detection with Yolo
            for (const auto &cbb : color_bboxes) {
                cv::Rect rcbb(cbb.x, cbb.y, cbb.w, cbb.h);
                cv::Rect roi_box = cv_alg::resizeBox(rcbb, 3);
                cv::Mat roi_img = cv_img(roi_box);
                yolo_bboxes = callYoloDetector(roi_img, hdr);
                for (const auto &ybb : yolo_bboxes) {
                    BBox true_bbox = {
                        roi_box.x + ybb.x,
                        roi_box.y + ybb.y,
                        ybb.w,
                        ybb.h,
                        ybb.prob,
                        ybb.obj_class
                    };
                    bboxes.push_back(true_bbox);
                }
            }
            break;

        default:
            ROS_ERROR("Unknown detection strategy");
    }
    return bboxes;
}


/* Callback for new camera input */
void MbzircDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::vector<BBox> bboxes;
    // darknet_ros_msgs::BoundingBoxes ros_bboxes; TODO old code
    distance_finder::ObjectBoxes ros_bboxes;

    ROS_INFO("Detector received a new input frame");
    // Change camera
    // switchCamera(short_camera_topic);
    
    // Yolo detector
    bboxes = detect(msg);

    // List bboxes (debug)
    ROS_INFO("Detected %d objects:", (int)bboxes.size());
    for (const auto &b : bboxes) {
        ROS_INFO("\t %s at [(%d,%d),(%d,%d)] with prob %f",
            b.obj_class.c_str(), b.x, b.y, b.x+b.w, b.y+b.h, b.prob    
        );
    }

    // Publish result (if non empty)
    if (bboxes_topic_enable && !bboxes.empty()) {
        for (const auto &b : bboxes) {
            /* TODO old code
            darknet_ros_msgs::BoundingBox ros_bbox;
            ros_bbox.Class = b.obj_class;
            ros_bbox.prob = b.prob;
            ros_bbox.x = b.x;
            ros_bbox.y = b.y;
            ros_bbox.w = b.w;
            ros_bbox.h = b.h;
            */
            distance_finder::ObjectBox ros_bbox;
            ros_bbox.obj_class = b.obj_class;
            ros_bbox.x = b.x + (b.w/2);
            ros_bbox.y = b.y + (b.h/2);
            ros_bbox.w = b.w;
            ros_bbox.h = b.h;
            ros_bboxes.obj_boxes.push_back(ros_bbox);
        }
        bboxes_pub_.publish(ros_bboxes);
    }
    if (det_img_topic_enable) {
        cv_bridge::CvImagePtr det_img_ptr;
        // Get raw image
        try {
            det_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& cve) {
            ROS_ERROR("cv_bridge: error converting img to OpenCV: %s", cve.what());
        }
        // Draw bounding boxes
        for (const auto &b : bboxes) {
            cv::rectangle(det_img_ptr->image, cv::Rect(b.x, b.y, b.w, b.h),
                cv::Scalar(100, 200, 0), 2
            );
        }
        sensor_msgs::ImagePtr det_img_msg = cv_bridge::CvImage(std_msgs::Header(), 
            "bgr8", det_img_ptr->image).toImageMsg();
        det_img_pub_.publish(det_img_msg);
    }
}

