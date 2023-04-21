#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <chrono>
#include <raspicam/raspicam_cv.h>

cv::Size imgSize;
cv::Mat image;

int h = 480, w = 640;

cv::Mat cv_image;
cv::Mat image_raw;

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/automobile/image_raw", 1);
    
    cv_image = cv::Mat::zeros(480, 640, CV_8UC3); // set the size to match image_raw
    image = cv::Mat::zeros(480, 640, CV_8UC3);

    raspicam::RaspiCam_Cv camera_;

    if (!camera_.open()) {
        ROS_ERROR("Failed to open the camera.");
        return 1;
    }

    camera_.set(cv::CAP_PROP_FPS,50);
    // camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    // camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // // Set the region of interest (ROI) to maintain the aspect ratio and field of view
    // camera_.set(cv::CAP_PROP_ROI_X, 0.0);
    // camera_.set(cv::CAP_PROP_ROI_Y, 0.0);
    // camera_.set(cv::CAP_PROP_ROI_WIDTH, 1.0);
    // camera_.set(cv::CAP_PROP_ROI_HEIGHT, 1.0);

    // image_raw=cv::Mat::zeros(960, 1280, CV_8UC3);
    // camera_.grab();
    // camera_.retrieve(image_raw);
    // std::cout << "cv_image size: " << cv_image.size() << std::endl;
    // cv::resize(image_raw, cv_image, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
    // std::cout << "cv_image size: " << cv_image.size() << std::endl;

    while(ros::ok()) {
        camera_.grab();
        camera_.retrieve(cv_image);
        // cv::resize(image_raw, cv_image, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
        // std::cout << "cv_image size: " << cv_image.size() << std::endl;
        sensor_msgs::ImagePtr modified_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
        image_pub.publish(modified_msg);
    }

    camera_.release();
    return 0;
}