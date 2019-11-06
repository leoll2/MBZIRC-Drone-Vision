#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>
#include <distance_finder/DistanceFinder.hpp>

/*void shutdownHandler(int sig)
{
    ROS_INFO("Shutting down DistanceFinder after receiving signal %d", sig);

    // Shutdown regardless of the signal type
    ros::shutdown();
}
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_finder");
    ros::NodeHandle nodeHandle("~");
    distance_finder::DistanceFinder distance_finder(nodeHandle);

    ros::spin();

    return 0;
}
