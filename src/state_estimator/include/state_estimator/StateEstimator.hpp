#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

class StateEstimator
{
    ros::Handle nh_;
    // TODO

public:
    StateEstimator(ros::NodeHandle nh);
    ~StateEstimator();
}
