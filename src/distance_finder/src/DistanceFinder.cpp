#include "ros/console.h"
#include "distance_finder/DistanceFinder.hpp"

#define RAD2DEG 57.29578

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

    std_msgs::Header header = dist_act_ptr->header;
    std::string cam_name = dist_act_ptr->cam_name;
    bool use_dmap = dist_act_ptr->use_dmap;
    uint32_t obj_x = dist_act_ptr->x;
    uint32_t obj_y = dist_act_ptr->y;
    uint32_t obj_w = dist_act_ptr->w;
    uint32_t obj_h = dist_act_ptr->h;
    std::string obj_class("primary_target"); // TODO: fixare l'azione

    // Compute distance and error
    PosError pe = findPosError(cam_name, obj_x, obj_y, obj_w, obj_h, obj_class, header);

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
double DistanceFinder::findDistanceByProportion(const CameraParameters& cp, 
    uint32_t x, uint32_t y, uint32_t w, uint32_t h, std::string obj_class)
{
    double obj_radius;

    if (obj_class.compare("primary_target") == 0)
        obj_radius = fly_ball_params.radius;
    else if (obj_class.compare("secondary_target") == 0)
        obj_radius = gnd_ball_params.radius;
    else
        ROS_WARN("Unrecognized class in findDistanceByProportion");

    // Observed flying ball radius in pixel (usually the largest dimension is closer to reality)
    double obj_radius_pix = std::max(w, h) / 2;

    return (cp.focal_len * obj_radius * cp.resolution_width) / 
        (cp.sensor_width*obj_radius_pix);
}


/* Compute the distance from the object by the means of a stereocamera depth map.
 * Returns a negative number if cannot extract info from the dmap for some reason */
double DistanceFinder::findDistanceByDepthMap(const CameraParameters& cp, 
    uint32_t x, uint32_t y, uint32_t w, uint32_t h, ros::Time ts)
{
    // this ugly trick is needed because getElementAfterTime is not inclusive
    ros::Duration one_nanosecond(0, 1);
    ros::Time search_timestamp = ts - one_nanosecond;

    // Get the latest depth map published at (or before) the image timestamp
    sensor_msgs::ImageConstPtr dmap_ptr = dmap_cache_.getElemAfterTime(search_timestamp);
    if (!dmap_ptr) {
        dmap_ptr = dmap_cache_.getElemBeforeTime(ts);
        if (! dmap_ptr) {
            return -1.0;
        }
    }
    
    if (dmap_ptr->header.stamp + ros::Duration(0, 50000000) < ts) {
        ROS_WARN("Latest depth map available is older than 50 ms ago, ignoring it");
        return -1.0;
    }

    // Convert the depth map to OpenCV format
    cv_bridge::CvImageConstPtr cv_dmap = cv_bridge::toCvCopy(dmap_ptr, 
        sensor_msgs::image_encodings::TYPE_32FC1
    );

    // Given the bounding box, read the depth from 9 internal points.
    // Sort them, then take the second minimum.
    // This is possibly better than taking the minimum as it may be a spurious outlier
    std::vector<float> depths;
    int x_ref, y_ref;
    float d_ref;
    int refs_half = 1;   // n_refs = (2*refs_half + 1)^2  TODO make this configurable
   
    for (int i=-refs_half; i <= refs_half; ++i) {
        for (int j=-refs_half; j <= refs_half; ++j) {
            x_ref = int(x + j*(int)w/2/(refs_half+1));
            y_ref = int(y + i*(int)h/2/(refs_half+1));
            if ((x_ref >= 0) && (x_ref < cp.resolution_width) && 
                (y_ref >= 0) && (y_ref < cp.resolution_height)) 
            {
                d_ref = cv_dmap->image.at<float>(y_ref, x_ref);
                if (std::isnormal(d_ref))
                    depths.push_back(d_ref);
            } else {
                ROS_WARN("Attempted out-of-index dmap access at (%d,%d)", x_ref, y_ref);
            }
        }
    }

    std::sort(depths.begin(), depths.end());

    if (depths.size() > 1) {
        return depths[1];
    } else if (depths.size() == 1) {
        return depths[0];
    } else
        return -1.0;
}


/* Retrieve the camera orientation (if any camera features an IMU).
 * Return (0,0,0) if  */
RPY DistanceFinder::findOrientation(ros::Time ts)
{
    RPY orient = {0, 0, 0};

    // this ugly trick is needed because getElementAfterTime is not inclusive
    ros::Duration one_nanosecond(0, 1);
    ros::Time search_timestamp = ts - one_nanosecond;

    geometry_msgs::PoseStampedConstPtr pose_ptr = pose_cache_.getElemAfterTime(search_timestamp);
    if (! pose_ptr) {
        pose_ptr = pose_cache_.getElemBeforeTime(ts);
        if (! pose_ptr) {
            return orient;
        }
    }

    if (pose_ptr->header.stamp + ros::Duration(0, 50000000) < ts) {
        ROS_WARN("Latest pose available is older than 50 ms ago, ignoring it");
        return orient;
    }

    double tx = pose_ptr->pose.position.x;
    double ty = pose_ptr->pose.position.y;
    double tz = pose_ptr->pose.position.z;

    // orientation quaternion
    tf2::Quaternion q(
        pose_ptr->pose.orientation.x,
        pose_ptr->pose.orientation.y,
        pose_ptr->pose.orientation.z,
        pose_ptr->pose.orientation.w
    );

    // rotation matrix
    tf2::Matrix3x3 m(q);

    m.getRPY(orient.roll, orient.pitch, orient.yaw);

    return orient;
}


/* Compute the position error of the object (w.r.t. to the current viewpoint).
 * The distance is computed by the means of a depth map (when available) or 
 * a simple geometrical proportion. */
PosError DistanceFinder::findPosError(std::string cam_name, 
    uint32_t x, uint32_t y, uint32_t w, uint32_t h, std::string obj_class, std_msgs::Header header)
{
    tf2::Quaternion q_corr;
    RPY orient;
    PosError pe;
    double dist = 50.0, dist_y;
    double x_m_raw, y_m_raw, x_m_pitched, y_m_pitched;
    double gamma, delta;

    try {
        const CameraParameters &cp = cam_params.at(cam_name);
        orient = findOrientation(header.stamp);

        if (cp.stereo) {
            // Use depth map if available (stereo camera)
            double dist_dmap, dist_prop;
            dist_dmap = findDistanceByDepthMap(cp, x, y, w, h, header.stamp);
            dist_prop = findDistanceByProportion(cp, x, y, w, h, obj_class);
            if (dist_dmap < 0) {
                // if dmap was unreadable, fallback to prop dist
                dist = dist_prop;
                ROS_INFO("Using dist_prop (dmap unreadable)");
            } else {
                // if dist_dmap is available
                if (0.75 * dist_dmap < dist_prop) {
                    // prefer depth map (more accurate) even if slightly larger than prop dist
                    dist = dist_dmap;
                    ROS_INFO("Using dist_dmap");
                } else {
                    // choose the minimum (as dist_prop is almost never larger the actual object)
                    dist = std::min(dist_dmap, dist_prop);
                    ROS_INFO("Using dist_min (%.2f, %.2f)", dist_dmap, dist_prop);
                }
            }
        } else {
            dist = findDistanceByProportion(cp, x, y, w, h, obj_class);
            ROS_INFO("Using dist_prop (not stereo)");
        }
    
        pe.dist_m = dist;
        // position error in pixel (w/o orientation correction)
        pe.x_pix = signed(x - (cp.resolution_width/2));
        pe.y_pix = signed(y - int(cp.resolution_height/2));
        // position error in metres (w/o orientation adjustment)
        x_m_raw = pe.x_pix * ((cp.calib_fov_width * dist / cp.calib_dist) / (double)cp.resolution_width);
        y_m_raw = pe.y_pix * ((cp.calib_fov_height * dist / cp.calib_dist) / (double)cp.resolution_height);
        //z_m_raw = sqrt(dist*dist - x_m_raw*x_m_raw - y_m_raw*y_m_raw);
        // Note that our (x,-y,z) coordinates actually map to (y,z,x), since x is the roll axis, y the pitch, z the yaw
        // Roll compensation
        tf2::Vector3 err_m_raw(0, x_m_raw, y_m_raw);
        q_corr.setRPY(orient.roll, 0, 0);
        q_corr.normalize();
        tf2::Vector3 err_m_corr = tf2::quatRotate(q_corr, err_m_raw);
        x_m_pitched = err_m_corr.y();
        y_m_pitched = err_m_corr.z();
        // Pitch compensation // TODO to be fixed
        /*
        dist_y = std::sqrt(std::max(0.0001, dist*dist - x_m_pitched*x_m_pitched));
        gamma = std::asin(y_m_pitched / dist_y);
        delta = orient.pitch - gamma;
        pe.y_m = dist_y * std::sin(delta);
        */
        pe.y_m = y_m_pitched;
        pe.x_m = x_m_pitched;

        ROS_DEBUG("Roll: %.2f  Pitch:%.2f  Yaw:%.2f", orient.roll*RAD2DEG, orient.pitch*RAD2DEG, orient.yaw*RAD2DEG);
        ROS_DEBUG("x_m_raw =%f  y_m_raw =%f", x_m_raw, y_m_raw);
        ROS_DEBUG("x_m_corr=%f  y_m_corr=%f", pe.x_m, pe.y_m);
    } catch (const std::out_of_range& e) {
        ROS_ERROR("Could not find camera parameters; is %s present in the configuration file?", cam_name.c_str());
    }
    return pe;
}


/* Constructor */
DistanceFinder::DistanceFinder(ros::NodeHandle nh)  // TODO reduce cache size to minimum
  : nh_(nh),
    dmap_sub_(nh_, "/distance_finder/depth_map", 1),
    pose_sub_(nh_, "/distance_finder/pose", 1),
    dmap_cache_(dmap_sub_, 30),
    pose_cache_(pose_sub_, 30),
    dist_act_srv_(nh_, "get_distance",
        boost::bind(&DistanceFinder::getDistanceActionGoalCallback,
            this, _1
        ), false
    )
{
    readROSParameters();
    initTargetParameters();
    initCamParameters();
    initDistanceActionServer();

    // Publish position
    target_pos_pub_ = nh_.advertise<distance_finder::ObjPosVec>(
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
void DistanceFinder::bboxesCallback(const distance_finder::ObjectBoxes::ConstPtr& obj_boxes_ptr)
{
    distance_finder::ObjPosVec tpos_vec;
    std::string cam_name;
    
    // Extract the camera name (and make sure it exists) and relative parameters
    cam_name = obj_boxes_ptr->cam_name;
    if (cam_params.find(cam_name) == cam_params.end())
        ROS_ERROR("Can't estimate distance from an unknown camera image (%s)", cam_name.c_str());
    const CameraParameters &cp = cam_params.at(cam_name);

    // Extract and reuse the header (to have the same timestamp)
    tpos_vec.header = obj_boxes_ptr->header;

    for (const auto &obj : obj_boxes_ptr->obj_boxes) {
        distance_finder::PosError poserr;
        distance_finder::ObjPos tpos;
        // Compute the distance and position errors
        poserr = findPosError(cam_name, obj.x, obj.y, obj.w, obj.h, obj.obj_class, tpos_vec.header);
        tpos.obj_class = obj.obj_class;
        tpos.res_w = (unsigned)cp.resolution_width;
        tpos.res_h = (unsigned)cp.resolution_height;
        tpos.dist = poserr.dist_m;
        tpos.err_x_m = poserr.x_m;
        tpos.err_y_m = poserr.y_m;
        tpos.err_x_pix = poserr.x_pix;
        tpos.err_y_pix = poserr.y_pix;
        tpos_vec.targets_pos.push_back(tpos);
        ROS_INFO("distance: %.2f  err_X=%.2f  err_Y=%.2f", tpos.dist, tpos.err_x_m, tpos.err_y_m);
    }

    //if (obj_boxes_ptr->obj_boxes.size() == 0)
    //    ROS_INFO("No object found");

    // Publish the result
    target_pos_pub_.publish(tpos_vec);
}

} /* end of distance_finder namespace */
