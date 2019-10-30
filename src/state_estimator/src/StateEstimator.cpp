#include "ros/console.h"
#include "state_estimator/StateEstimator.hpp"


/* Constructor */
StateEstimator::StateEstimator(ros::NodeHandle nh)
  : nh_(nh)
{
    
}


/* Destructor */
StateEstimator::~StateEstimator() {}
