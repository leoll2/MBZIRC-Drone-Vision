#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>
#include <mbzirc_detector/MbzircDetector.hpp>

/*void shutdownHandler(int sig)
{
    ROS_INFO("Shutting down StateEstimator after receiving signal %d", sig);

    // Shutdown regardless of the signal type
    ros::shutdown();
}
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_estimator");
    ros::NodeHandle("~");
    StateEstimator state_estimator(nodeHandle);

    ros::spin();

    return 0;
}
