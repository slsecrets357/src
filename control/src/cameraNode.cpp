#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <chrono>
#include <raspicam/raspicam_cv.h>
#include "/home/pi/Documents/Brain_ROS/devel/include/utils/Lane.h"
#include "/home/pi/Documents/Brain_ROS/devel/include/utils/Sign.h"
#include "include/yolo-fastestv2.h"
using namespace std::chrono;

// yoloFastestv2 api;
// ros::Publisher sign_pub;
// ros::Publisher lane_pub;
cv::Size imgSize;
bool stopline;
cv::Mat image;

int h = 480, w = 640;

double minVal, maxVal;
cv::Point minLoc, maxLoc;
cv::Mat img_gray;
cv::Mat img_roi;
cv::Mat thresh;
cv::Mat hist;
cv::Mat img_rois;
double threshold_value_stop;
cv::Mat threshs;
cv::Mat hists;
cv::Mat cv_image;

static const char* class_names[] = {
        "oneway", "highwayentrance", "stopsign", "roundabout", "park", "crosswalk", "noentry", "highwayexit", "priority",
                "lights","block","pedestrian","car"
    };

std::vector<int> extract_lanes(cv::Mat hist_data) {
    std::vector<int> lane_indices;
    int previous_value = 0;
    for (int idx = 0; idx < hist_data.cols; ++idx) {
        int value = hist_data.at<int>(0, idx);
        if (value >= 1500 && previous_value == 0) {
            lane_indices.push_back(idx);
            previous_value = 255;
        } else if (value == 0 && previous_value == 255) {
            lane_indices.push_back(idx);
            previous_value = 0;
        }
    }
    if (lane_indices.size() % 2 == 1) {
        lane_indices.push_back(640 - 1);
    }
    return lane_indices;
}

double optimized_histogram(cv::Mat image, bool show = false) {
    stopline = false;
    cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY);

    img_roi = img_gray(cv::Rect(0, 384, 640, 96));
    cv::minMaxLoc(img_roi, &minVal, &maxVal, &minLoc, &maxLoc);
    double threshold_value = std::min(std::max(maxVal - 55.0, 30.0), 200.0);
    cv::threshold(img_roi, thresh, threshold_value, 255, cv::THRESH_BINARY);
    hist = cv::Mat::zeros(1, w, CV_32SC1);
    cv::reduce(thresh, hist, 0, cv::REDUCE_SUM, CV_32S);

    std::vector<int> lanes = extract_lanes(hist);
    std::vector<double> centers;
    for (size_t i = 0; i < lanes.size() / 2; ++i) {
        if (abs(lanes[2 * i] - lanes[2 * i + 1])>350 && threshold_value>50){
            stopline = true;
            if (!show) return w / 2.0;
        }
        if (3 < abs(lanes[2 * i] - lanes[2 * i + 1])) {
            centers.push_back((lanes[2 * i] + lanes[2 * i + 1]) / 2.0);
        }
    }
    double center;
    if (centers.empty()) {
        center = w / 2.0;
    } else if (centers.size() == 1) {
        center = (centers[0] > (w / 2.0)) ? (centers[0] - 0) / 2 : (centers[0] * 2 + w) / 2;
    } else if (abs(centers[0] - centers.back()) < 200) {
        center = ((centers[0] + centers.back()) > w) ? ((centers[0] + centers.back()) / 2 + 0) / 2.0 : ((centers[0] + centers.back()) + w) / 2;
    } else {
        center = (centers[0] + centers.back()) / 2;
    }

    if (show) {
        // Create the new cv::Mat object and initialize it with zeros
        cv::Mat padded_thresh = cv::Mat::zeros(480, 640, CV_8UC1);

        // Copy the truncated array into the new cv::Mat object
        cv::Mat roi = padded_thresh(cv::Range(384, 384+thresh.rows), cv::Range::all());
        thresh.copyTo(roi);
        if (stopline) {
            cv::putText(padded_thresh, "Stopline detected!", cv::Point(static_cast<int>(w * 0.5), static_cast<int>(h * 0.5)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
        cv::line(image, cv::Point(static_cast<int>(center), image.rows), cv::Point(static_cast<int>(center), static_cast<int>(0.8 * image.rows)), cv::Scalar(0, 0, 255), 5);
        cv::Mat add;
        cv::cvtColor(padded_thresh, add, cv::COLOR_GRAY2BGR);
        cv::imshow("Lane", image + add);
        cv::waitKey(1);
    }
    return center;
}

static void laneDetectionCallback(const ros::TimerEvent& event, ros::Publisher *pub) {
    auto start = high_resolution_clock::now();
    double center = optimized_histogram(cv_image);
    utils::Lane lane_msg;
    lane_msg.center = center;
    lane_msg.stopline = stopline;
    lane_msg.header.stamp = ros::Time::now();
    pub->publish(lane_msg);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "lane durations: " << duration.count() << std::endl;
}
static void signDetectionCallback(const ros::TimerEvent& event, yoloFastestv2 *api, ros::Publisher *pub) {
    auto start = high_resolution_clock::now();
    std::vector<TargetBox> boxes;
    api->detection(cv_image, boxes);

    utils::Sign sign_msg;
    sign_msg.header.stamp = ros::Time::now();
    // sign_msg.header.frame_id = "camera_frame";

    sign_msg.num = boxes.size();

    for (const auto &box : boxes) {
        sign_msg.objects.push_back(box.cate);
        sign_msg.box1.push_back(box.x1);
        sign_msg.box1.push_back(box.y1);
        sign_msg.box2.push_back(box.x2);
        sign_msg.box2.push_back(box.y2);
        sign_msg.confidence.push_back(box.score);
    }
    // Publish Sign message
    pub->publish(sign_msg);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "sign durations: " << duration.count() << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");
    yoloFastestv2 api;
    api.loadModel("/home/pi/Documents/Brain_ROS/src/control/src/model/alice7s-opt.param",
                  "/home/pi/Documents/Brain_ROS/src/control/src/model/alice7s-opt.bin");
    ros::NodeHandle nh;
    ros::Publisher lane_pub = nh.advertise<utils::Lane>("lane", 3);
    ros::Publisher sign_pub = nh.advertise<utils::Sign>("sign", 3);
    double lane_pub_rate = 5.0; // Adjust this value for lane_pub rate
    double sign_pub_rate = 2.0;
    ros::Timer lane_timer = nh.createTimer(ros::Duration(1.0 / lane_pub_rate), [&](const ros::TimerEvent& event) { laneDetectionCallback(event, &lane_pub); });
    ros::Timer sign_timer = nh.createTimer(ros::Duration(1.0 / sign_pub_rate), [&](const ros::TimerEvent& event) { signDetectionCallback(event, &api, &sign_pub); });

    raspicam::RaspiCam_Cv camera_;
    // camera_.set(cv::CAP_PROP_FRAME_WIDTH, imgSize.width);
    // camera_.set(cv::CAP_PROP_FRAME_HEIGHT, imgSize.height);
    // camera_.set(cv::CAP_PROP_FPS, 15);
    // camera_.set(cv::CAP_PROP_BRIGHTNESS, 42);
    // camera_.set(cv::CAP_PROP_FORMAT, CV_8UC3);

    if (!camera_.open()) {
        ROS_ERROR("Failed to open the camera.");
        return 1;
    }

    image = cv::Mat::zeros(480, 640, CV_8UC1);
    stopline = false;

    while(ros::ok()) {
        camera_.grab();
        camera_.retrieve(cv_image);
        // cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
        cv::imshow("Camera", cv_image);
        cv::waitKey(1);
        ros::spinOnce();
    }
    camera_.release();
    return 0;
}
