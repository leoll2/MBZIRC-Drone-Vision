#pragma once

#include <map>
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <actionlib/server/simple_action_server.h>

#include <distance_finder/ObjectBox.h>
#include <distance_finder/ObjectBoxes.h>
#include <distance_finder/TargetPos.h>
#include <distance_finder/TargetPosVec.h>
#include <distance_finder/GetDistanceAction.h>

namespace distance_finder {

enum DistanceAlgorithm {PROPORTION, DEPTH_MAP};


typedef struct TargetParameters {
    std::string color;
    double radius;
} TargetParameters;


typedef struct CameraParameters {
    int resolution_width;
    int resolution_height; 
    double focal_len;
    double sensor_width;
    double sensor_height;
    double sensor_diag;
    double calib_dist;
    double calib_fov_width;
    double calib_fov_height;
    bool stereo;
} CameraParameters;


typedef struct RPY {
    double roll;
    double pitch;
    double yaw;
} RPY;


typedef struct PosError {
    int x_pix;
    int y_pix;
    double x_m;
    double y_m;
    double dist_m;
} PosError;


class DistanceFinder
{
    ros::NodeHandle nh_;
    ros::Subscriber bbox_sub_;
    ros::Publisher target_pos_pub_;
    message_filters::Subscriber<sensor_msgs::Image> dmap_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    message_filters::Cache<sensor_msgs::Image> dmap_cache_;
    message_filters::Cache<geometry_msgs::PoseStamped> pose_cache_;
    actionlib::SimpleActionServer<distance_finder::GetDistanceAction> dist_act_srv_;
    std::map<std::string, CameraParameters> cam_params;
    TargetParameters fly_ball_params;
    TargetParameters gnd_ball_params;
    std::string input_bbox_topic;
    std::string target_pos_topic;
    int target_pos_q_size;
    bool target_pos_latch;

    void readROSParameters();
    void initTargetParameters();
    void initCamParameters();
    void initDistanceActionServer();
    void getDistanceActionGoalCallback(const distance_finder::GetDistanceGoalConstPtr &dist_act_ptr);
    void getDistanceActionPreemptCallback();
    RPY findOrientation(ros::Time timestamp);
    double findDistanceByProportion(const CameraParameters& cp, uint32_t x, uint32_t y, uint32_t w, uint32_t h);
    double findDistanceByDepthMap(const CameraParameters& cp, uint32_t x, uint32_t y, uint32_t w, uint32_t h, ros::Time timestamp);
    PosError findPosError(std::string cam_name, uint32_t x, uint32_t y, uint32_t w, uint32_t h, std_msgs::Header header);


public:
    DistanceFinder(ros::NodeHandle nh);
    ~DistanceFinder();
    void bboxesCallback(const distance_finder::ObjectBoxes::ConstPtr& obj_boxes_ptr);
};

}
