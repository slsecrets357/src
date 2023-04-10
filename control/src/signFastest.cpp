#include "ros/ros.h"
#include "include/yolo-fastestv2.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include "/home/pi/Documents/Brain_ROS/devel/include/utils/Sign.h"
#include <chrono>
using namespace std::chrono;

static const char* class_names[] = {
        "oneway", "highwayentrance", "stopsign", "roundabout", "park", "crosswalk", "noentry", "highwayexit", "priority",
                "lights","block","pedestrian","car"
    };
void imageCallback(const sensor_msgs::ImageConstPtr &msg, yoloFastestv2 *api, ros::Publisher *pub) {
    auto start = high_resolution_clock::now();

    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Perform object detection
    std::vector<TargetBox> boxes;
    api->detection(cv_ptr->image, boxes);

    // Fill Sign message with all detected objects
    utils::Sign sign_msg;
    sign_msg.header.stamp = ros::Time::now();
    sign_msg.header.frame_id = "camera_frame"; // Set to appropriate frame_id if needed

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
    
    // for display
    // for (int i = 0; i < boxes.size(); i++) {
    //     // std::cout << "hi" << std::endl;
    //     std::cout<<boxes[i].x1<<" "<<boxes[i].y1<<" "<<boxes[i].x2<<" "<<boxes[i].y2
    //              <<" "<<boxes[i].score<<" "<<boxes[i].cate<<std::endl;
        
    //     char text[256];
    //     sprintf(text, "%s %.1f%%", class_names[boxes[i].cate], boxes[i].score * 100);

    //     int baseLine = 0;
    //     cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    //     int x = boxes[i].x1;
    //     int y = boxes[i].y1 - label_size.height - baseLine;
    //     if (y < 0)
    //         y = 0;
    //     if (x + label_size.width > cv_ptr->image.cols)
    //         x = cv_ptr->image.cols - label_size.width;

    //     cv::rectangle(cv_ptr->image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
    //                   cv::Scalar(255, 255, 255), -1);

    //     cv::putText(cv_ptr->image, text, cv::Point(x, y + label_size.height),
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

    //     cv::rectangle (cv_ptr->image, cv::Point(boxes[i].x1, boxes[i].y1), 
    //                    cv::Point(boxes[i].x2, boxes[i].y2), cv::Scalar(255, 255, 0), 2, 2, 0);
    // }
    // cv::imshow("image", cv_ptr->image);
    // cv::waitKey(1);
}


int main(int argc, char **argv) {
    // ... (same as before, including class_names and yoloFastestv2 instance)
    double num_iterations = 1;
    double total;
    static const char* class_names[] = {
        "oneway", "highwayentrance", "stopsign", "roundabout", "park", "crosswalk", "noentry", "highwayexit", "priority",
                "lights","block","pedestrian","car"
    };
    yoloFastestv2 api;

    api.loadModel("/home/pi/Documents/Brain_ROS/src/control/src/model/yolo-fastestv2-opt.param",
                  "/home/pi/Documents/Brain_ROS/src/control/src/model/yolo-fastestv2-opt.bin");

    // Initialize ROS node and publisher
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Publisher pub = nh.advertise<utils::Sign>("sign", 1000);
    image_transport::Subscriber sub = it.subscribe("automobile/image_raw", 1, boost::bind(&imageCallback, _1, &api, &pub));

    // Spin ROS node
    ros::spin();

    return 0;
}
