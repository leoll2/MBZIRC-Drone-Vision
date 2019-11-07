#include "ros/console.h"
#include "distance_finder/DistanceFinder.hpp"

namespace distance_finder {

/* Read the ROS configuration from file */
void DistanceFinder::readROSParameters()
{
    nh_.getParam("subscribers/input_bbox/topic", input_bbox_topic);

    nh_.getParam("publishers/target_pos/topic", target_pos_topic);
    nh_.getParam("publishers/target_pos/queue_size", target_pos_q_size);
    nh_.getParam("publishers/target_pos/latch", target_pos_latch);
}


/* Initialize the parameters for each supported camera */
void DistanceFinder::initCamParameters()
{
    std::vector<std::string> cam_names;
    nh_.getParam("supported_cams", cam_names);

    for (const auto &cam : cam_names) {
        CameraParameters cp;

        nh_.getParam(cam + "/resolution/width", cp.resolution_width);
        nh_.getParam(cam + "/resolution/height", cp.resolution_height);
        nh_.getParam(cam + "/lens/focal_length", cp.focal_len);
        nh_.getParam(cam + "/sensor/width", cp.sensor_width);
        nh_.getParam(cam + "/sensor/height", cp.sensor_height);
        nh_.getParam(cam + "/sensor/diag", cp.sensor_diag);
        nh_.getParam(cam + "/calibration/distance", cp.calib_dist);
        nh_.getParam(cam + "/stereo", cp.stereo);

        cp.calib_fov_width = cp.sensor_width / cp.focal_len * cp.calib_dist;
        cp.calib_fov_height = cp.sensor_height / cp.focal_len * cp.calib_dist;

        cam_params[cam] = cp;
    }
}


/* Initialize the parameters for each target object */
void DistanceFinder::initTargetParameters()
{
    nh_.getParam("target/flying_ball/radius", fly_ball_params.radius);
    nh_.getParam("target/flying_ball/color", fly_ball_params.color);
    nh_.getParam("target/ground_ball/radius", gnd_ball_params.radius);
    nh_.getParam("target/ground_ball/color", gnd_ball_params.color);
}


/* Initialize the GetDistance action server */
void DistanceFinder::initDistanceActionServer()
{
    dist_act_srv_.registerPreemptCallback(
        boost::bind(&DistanceFinder::getDistanceActionPreemptCallback,
        this)
    );
    dist_act_srv_.start();
}


/* GetDistance action goal callback */
void DistanceFinder::getDistanceActionGoalCallback(const distance_finder::GetDistanceGoalConstPtr &dist_act_ptr)
{
    distance_finder::GetDistanceResult dist_act_res;

    std::string cam_name = dist_act_ptr->cam_name;
    bool use_dmap = dist_act_ptr->use_dmap;
    uint32_t obj_x = dist_act_ptr->x;
    uint32_t obj_y = dist_act_ptr->y;
    uint32_t obj_w = dist_act_ptr->w;
    uint32_t obj_h = dist_act_ptr->h;

    // Compute distance and error
    PosError pe = findPosErrorByProportion(cam_name, obj_x, obj_y, obj_w, obj_h);

    // Produce action response
    dist_act_res.dist = pe.dist_m;
    dist_act_res.err_x_m = pe.x_m;
    dist_act_res.err_y_m = pe.y_m;
    dist_act_srv_.setSucceeded(dist_act_res);
}


/* GetDistance action preempt callback */
void DistanceFinder::getDistanceActionPreemptCallback()
{
    ROS_DEBUG("Inside GetDistance action preempt callback");
    // TODO
}


/* Compute the distance from the object by the means of a simple proportion */
double DistanceFinder::findDistanceByProportion(std::string cam_name, 
    uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    CameraParameters &cp = cam_params[cam_name];

    // Observed flying ball radius in pixel
    double fly_ball_radius_pix = (w + h)/2;
    // flying ball radius in pixel at calibration distance
    // TODO is the number 800 correct? where is it from?
    double calib_fly_ball_radius_pix = 800.0 / cp.calib_fov_width * fly_ball_params.radius;

    return cp.calib_dist * calib_fly_ball_radius_pix / fly_ball_radius_pix;
}


/* Compute the position error of the object (w.r.t. to the current viewpoint)
 * by the means of a simple proportion */
PosError DistanceFinder::findPosErrorByProportion(std::string cam_name, 
    uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    CameraParameters &cp = cam_params[cam_name];
    PosError pe;

    double dist = findDistanceByProportion(cam_name, x, y, w, h);
    pe.dist_m = dist;
    pe.x_pix = x - (cp.resolution_width/2);
    pe.y_pix = y - (cp.resolution_height/2);
    pe.x_m = pe.x_pix * ((cp.calib_fov_width * dist / cp.calib_dist) / (double)cp.resolution_width);
    pe.y_m = pe.y_pix * ((cp.calib_fov_height * dist / cp.calib_dist) / (double)cp.resolution_height);

    return pe;
}


/* Constructor */
DistanceFinder::DistanceFinder(ros::NodeHandle nh)
  : nh_(nh),
    dist_act_srv_(nh_, "get_distance",
        boost::bind(&DistanceFinder::getDistanceActionGoalCallback,
            this, _1
        ), false
    )
{
    initTargetParameters();
    initCamParameters();
    initDistanceActionServer();

    // Publish position
    target_pos_pub_ = nh_.advertise<distance_finder::TargetPos>(
        target_pos_topic, target_pos_q_size, target_pos_latch
    );

    // Subscribe to incoming bounding boxes
    bbox_sub_ = nh_.subscribe(input_bbox_topic, 1,
        &DistanceFinder::bboxesCallback, this
    );

    ROS_INFO("DistanceFinder node fully initialized");
}


/* Destructor */
DistanceFinder::~DistanceFinder() {}


/* Callback for new bounding box received */
void DistanceFinder::bboxesCallback(const distance_finder::ObjectBoxes::ConstPtr& msg)
{
    ROS_INFO("Inside bboxesCallback");    // TODO DEBUG only
    //TODO
}

} /* end of distance_finder namespace */
