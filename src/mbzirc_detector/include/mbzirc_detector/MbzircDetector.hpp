#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>

#include <actionlib/client/simple_action_client.h>

#include <darknet_ros_msgs/DetectObjectsAction.h>

typedef struct BBox {
    int x;
    int y;
    int w;
    int h;
    float prob;
    std::string obj_class;
} BBox;

class MbzircDetector {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::NodeHandle mux_nh_;
    ros::ServiceClient cam_sel_client;
    actionlib::SimpleActionClient<darknet_ros_msgs::DetectObjectsAction> yolo_act_cl_;

    std::string input_camera_topic;
    std::string long_camera_name;
    std::string long_camera_topic;
    bool long_camera_stereo;
    std::string short_camera_name;
    std::string short_camera_topic;
    bool short_camera_stereo;

    void readParameters();
    void switchCamera(std::string camera_topic);
    std::vector<BBox> callYoloDetector(const sensor_msgs::ImageConstPtr& msg);
public:
    MbzircDetector(ros::NodeHandle nh);
    ~MbzircDetector();
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
};

