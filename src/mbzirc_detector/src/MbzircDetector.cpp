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

    nh_.getParam("long_cam_name", long_camera_name);
    nh_.getParam("cameras/" + long_camera_name + "/topic", long_camera_topic);
    nh_.getParam("cameras/" + long_camera_name + "/stereo", long_camera_stereo);
    nh_.getParam("short_cam_name", short_camera_name);
    nh_.getParam("cameras/" + short_camera_name + "/topic", short_camera_topic);
    nh_.getParam("cameras/" + short_camera_name + "/stereo", short_camera_stereo);

    nh_.getParam("calib_dir", calib_dir);
}


/* Initialize the color detector, according to its config file */
void MbzircDetector::initColorDetector()
{
    bool single_main_target;
    int luma_min, luma_max, a_min, a_max, b_min, b_max;
    int min_area_pix;
    int max_objects;
    int hu_metric;
    int hu_soft_hard_area_thresh;
    double hu_max_dist_soft, hu_max_dist_hard;

    nh_.getParam("color_detector/lab_thresh/luma_min", luma_min);
    nh_.getParam("color_detector/lab_thresh/luma_max", luma_max);
    nh_.getParam("color_detector/lab_thresh/a_min", a_min);
    nh_.getParam("color_detector/lab_thresh/a_max", a_max);
    nh_.getParam("color_detector/lab_thresh/b_min", b_min);
    nh_.getParam("color_detector/lab_thresh/b_max", b_max);

    nh_.getParam("color_detector/min_area_pix", min_area_pix);
    nh_.getParam("color_detector/max_objects", max_objects);
    nh_.getParam("color_detector/single_main_target", single_main_target);

    nh_.getParam("color_detector/hu/metric", hu_metric);
    nh_.getParam("color_detector/hu/soft_hard_area_thresh", hu_soft_hard_area_thresh);
    nh_.getParam("color_detector/hu/max_dist_soft", hu_max_dist_soft);
    nh_.getParam("color_detector/hu/max_dist_hard", hu_max_dist_hard);

    std::vector<unsigned> th {(unsigned)luma_min, (unsigned)luma_max, 
        (unsigned)a_min, (unsigned)a_max, (unsigned)b_min, (unsigned)b_max
    };
    color_detector = new ColorDetector(th, (unsigned)min_area_pix, (unsigned)max_objects, calib_dir, 
        single_main_target, (unsigned)hu_metric, (unsigned)hu_soft_hard_area_thresh, 
        hu_max_dist_soft, hu_max_dist_hard
    );
    ROS_INFO("Initialized color detector with thresholds=(%d,%d,%d,%d,%d,%d)\
        min_area_pix=%d max_objects=%d hu_metric=%d hu_area_thresh=%d hu_dist_soft=%f hu_dist_hard=%f", 
        luma_min, luma_max, a_min, a_max, b_min, b_max, min_area_pix, max_objects,
        hu_metric, hu_soft_hard_area_thresh, hu_max_dist_soft, hu_max_dist_hard
    );
}


/* Change the detector input channel, i.e. the videocamera to be read */
void MbzircDetector::switchCamera(CameraType new_cam_range)
{
    topic_tools::MuxSelect srv;

    if (new_cam_range == current_cam_range)
        return;

    switch (new_cam_range) {
        case SHORT_RANGE:
            srv.request.topic = short_camera_topic;
            break;
        case LONG_RANGE:
            srv.request.topic = long_camera_topic;
            break;
        default:
            ROS_ERROR("Attempting to switch to an unsupported camera mode");
    }

    current_cam_range = new_cam_range;

    if (cam_sel_client.call(srv)) {
        ROS_DEBUG("Switched camera from %s to %s", 
            srv.response.prev_topic.c_str(), srv.request.topic.c_str()
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
    current_cam_range(LONG_RANGE),
    det_strategy(COLOR)
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

    // Setup publisher for bounding boxes and detection image
    if (bboxes_topic_enable) {
        bboxes_pub_ = nh_.advertise<distance_finder::ObjectBoxes>( 
            bboxes_topic, bboxes_q_size, bboxes_latch
        );
    }
    if (det_img_topic_enable) {
        det_img_pub_ = nh_.advertise<sensor_msgs::Image>(
            det_img_topic, det_img_q_size, det_img_latch
        );
    }

    // Select the correct camera source
    switchCamera(current_cam_range);

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

        case COLOR:
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
            bboxes = callColorDetector(cv_img);
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


/* Map names used by the detector with those used by navigation code
 * TODO: maybe this can be moved to a configuration file*/
std::string MbzircDetector::det2nav_class(std::string det_class)
{
    if (det_class.compare("ball") == 0)
        return std::string("primary_target");
    else if (det_class.compare("drone") == 0)
        return std::string("drone");
    else {
        ROS_WARN("Unrecognized class name");
        return std::string("secondary_target");
    }
}


/* Callback for new camera input */
void MbzircDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::vector<BBox> bboxes;
    distance_finder::ObjectBoxes ros_bboxes;

    ROS_DEBUG("Detector received a new frame");
 
    // Find bounding boxes
    bboxes = detect(msg);

    // List bboxes (debug)
    ROS_DEBUG("Detected %d objects:", (int)bboxes.size());
    for (const auto &b : bboxes) {
        ROS_DEBUG("\t %s at [(%d,%d),(%d,%d)] with prob %f",
            b.obj_class.c_str(), b.x, b.y, b.x+b.w, b.y+b.h, b.prob    
        );
    }

    // Publish result (even if empty, for compatibility reasons)
    if (bboxes_topic_enable) {
        for (const auto &b : bboxes) {
            distance_finder::ObjectBox ros_bbox;
            ros_bbox.obj_class = det2nav_class(b.obj_class);
            ros_bbox.x = b.x + (b.w/2);
            ros_bbox.y = b.y + (b.h/2);
            ros_bbox.w = b.w;
            ros_bbox.h = b.h;
            ros_bboxes.obj_boxes.push_back(ros_bbox);
        }
        ros_bboxes.header = msg->header;
        ros_bboxes.cam_name = (current_cam_range == SHORT_RANGE) 
            ? short_camera_name : long_camera_name;
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
        sensor_msgs::ImagePtr det_img_msg = cv_bridge::CvImage(
            msg->header, 
            "bgr8", 
            det_img_ptr->image
        ).toImageMsg();
        det_img_pub_.publish(det_img_msg);
    }

    // TODO must implement a policy for this
    switchCamera(SHORT_RANGE);
}

