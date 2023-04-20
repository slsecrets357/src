#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <chrono>
#include <raspicam/raspicam_cv.h>
// using namespace std::chrono;

cv::Size imgSize;
cv::Mat image;

int h = 480, w = 640;

cv::Mat cv_image;

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    ros::Publisher image_pub = nh.advertise("/automobile/image_raw", 1);
    
    cv_image = cv::Mat::zeros(480, 640, CV_8UC3);
    image = cv::Mat::zeros(480, 640, CV_8UC3);

    raspicam::RaspiCam_Cv camera_;

    if (!camera_.open()) {
        ROS_ERROR("Failed to open the camera.");
        return 1;
    }

    while(ros::ok()) {
        camera_.grab();
        camera_.retrieve(cv_image);
        sensor_msgs::ImagePtr modified_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
        image_pub->publish(modified_msg);
        ros::spinOnce();
    }

    camera_.release();
    return 0;
}