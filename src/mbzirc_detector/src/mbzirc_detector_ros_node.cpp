#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>
#include <mbzirc_detector/MbzircDetector.hpp>

void shutdownHandler(int sig)
{
    ROS_INFO("Shutting down MbzircDetector after receiving signal %d", sig);

    // Shutdown regardless of the signal type
    ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mbzirc_detector");
    ros::NodeHandle nodeHandle("~");
    MbzircDetector detector(nodeHandle);

    ros::spin();
    /* TODO: support multithreading (acquisition/processing), remove spin()
    signal(SIGINT, shutdownHandler);
    signal(SIGTERM, shutdownHandler);
    signal(SIGKILL, shutdownHandler);
    */

    return 0;
}
