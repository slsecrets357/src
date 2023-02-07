#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <cstdio>
using namespace std;

class CameraHandler{
    public:
        cv::Vec2f point;
        int res;
        int threshold;
        int minlength;
        cv::Vec2f error_p;
        float error_w;
        int t;
        cv::Mat imgL;
        int lane_center;
        bool stopline;
        bool dotline;
        cv::Mat cv_image;
        cv_bridge::CvImagePtr bridge;
        ros::Rate rate;
        ros::Subscriber image_sub;
        CameraHandler();
        void callback(const sensor_msgs::ImageConstPtr &msg);
        void trackbars();
        void save_object();
        void view();
        void changePx(int v);
        void changePy(int v);
        void changeRes(int v);
        void changeThreshold(int v);
        void changeMinlength(int v);
        void changePex(int v);
        void changePey(int v);
        void changeEw(int v);
        void histogram();
        void dotted_lines();
        void detect_lines();
};

CameraHandler::CameraHandler(){
    string filepath = __FILE__;
    string dir = filepath.substr(0,filepath.find_last_of("/\\"));
    string jsonpath = dir + "/json-dump.json";
    ifstream file(jsonpath);
    Json::Value data;
    Json::Reader reader;
    if(reader.parse(file,data)){
        cout<< data.toStyledString()<<endl;
    }else{
        cout<< "error parsing the file" <<endl;
    }
    file.close();

    this->point = cv::Vec2f(data["point"][0].asFloat(), data["point"][1].asFloat());
    this->res = data["res"].asInt();
    this->threshold = data["threshold"].asInt();
    this->minlength = data["minlength"].asInt();
    error_p = cv::Vec2f(data["error_p"][0].asFloat(), data["error_p"][1].asFloat());
    error_w = data["error_w"].asFloat();
    t = 3;
    this->imgL = cv::Mat::zeros(640,480,CV_8UC1);
    this->lane_center = 0;
    this->stopline = false;
    this->dotline = false;

    cv:cvNamedWindow("Frame preview",cv::WINDOW_NORMAL);
    cv:cvResizeWindow("Frame preview",960,720);
    this->bridge = cv_bridge::CvImage();
    cv:Mat cv_image = cv::Mat::zeros(640,480.CV_8UC3);
    ros::init(argc,argv,"CAMnod");
    ros::NodeHandle n;
    this->rate = ros::Rate(3);
    this->image_sub = n.subscribe("/automobile/image_raw",10,&CameraHandler::callback,this);
    ros::spin();
}

void CameraHandler::callback(const sensor_msgs::ImageConstPtr &msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_image = cv_ptr->image;
    histogram();
    cv::imshow("Frame Preview", imgL);
    int key = cv::waitKey(1);
}

// void CameraHandler::trackbars(){
//     string windowName = "Params";
//     cv::namedWindow(windowName, cv::WINDOW_NORMAL);
//     cv::resizeWindow(windowName, 480, 360);
//     cv::createTrackbar("Save", windowName, 0, 1, &(CameraHandler::save_object), this);
//     cv::createTrackbar("View", windowName, 0, 1, &CameraHandler::view, this);
//     cv::createTrackbar("point[0]", windowName, (int)point[0]*100, 100, &CameraHandler::changePx, this);
//     cv::createTrackbar("point[1]", windowName, point[1]*100, 100, &CameraHandler::changePy, this);
//     cv::createTrackbar("res", windowName, res, 100, &CameraHandler::changeRes, this);
//     cv::createTrackbar("threshold",windowName, this->threshold,200,&CameraHandler::changeThreshold, this);
//     cv::createTrackbar("minlength",windowName,this->minlength,100,&CameraHandler::changeMinlength, this);
//     cv::createTrackbar('error_p[0]',windowName,(int)this->error_p[0]*100,100,&CameraHandler::changePex, this);
//     cv::createTrackbar('error_p[1]',windowName,(int)this->error_p[1]*100,100,&CameraHandler::changePey, this);
//     cv::createTrackbar('error_w',windowName,(int)this->error_w*100,100,&CameraHandler::changeEw, this);
// }

void CameraHandler::save_object(){
    string filepath = __FILE__;
    string dir = filepath.substr(0,filepath.find_last_of("/\\"));
    string jsonpath = dir + "/json-dump.json";
    ifstream file(jsonpath);
    Json::Value data;
    data["point"] = Json::Value(Json::arrayValue);
    for (const auto &p : this->point) data["point"].append(p);
    data["res"] = res;
    data["threshold"] = threshold;
    data["minlength"] = minlength;
    data["error_p"] = Json::Value(Json::arrayValue);
    for (const auto &ep : error_p) data["error_p"].append(ep);
    data["error_w"] = error_w;

    std::ofstream file("json-dump.json");
    file << data;
    file.close();
    this->view();
}

void CameraHandler::view(){
    cout << "=========== REMOTE CONTROL ============" << endl;
    cout << "point[0]           " << this->point[0] << "     [L/J]" << endl;
    cout << "point[1]         " << this->point[1] << "     [I/K]" << endl;
    cout << "res              " << this->res << "     [1/2]" << endl;
    cout << "threshold        " << this->threshold << "     [3/4]" << endl;
    cout << "minlength        " << this->minlength << "     [5/6]" << endl;
    cout << "t                " << this->t << "     [6/7]" << endl;
    cout << "error_p[0]       " << this->error_p[0] << "     [h/f]" << endl;
    cout << "error_p[1]       " << this->error_p[1] << "     [g/t]" << endl;
    cout << "error_w          " << this->error_w << "     [e/r]" << endl;
    cout << "lane_center      " << this->lane_center/640 << endl;
    cout << "lane_left        " << this->left_lane/640 << endl;
    cout << "lane_right       " << this->right_lane/640 << endl;
}

void CameraHandler::changePx(int v){
    this->point[0] = v/100;
    this->detect_lines();
    string windowName = "Frame preview";
    cv::imshow(windowName, this->imgL);
}

void CameraHandler::changePy(int v){
    this->point[1] = v/100;
    this->detect_lines();
    string windowName = "Frame preview";
    cv::imshow(windowName, this->imgL);
}

void CameraHandler::changeRes(int v){
    this->res = v;
    this->detect_lines();
    string windowName = "Frame preview";
    cv::imshow(windowName, this->imgL);
}

void CameraHandler::changeThreshold(int v){
    this->threshold = v;
    this->detect_lines();
    string windowName = "Frame preview";
    cv::imshow(windowName, this->imgL);
}

void CameraHandler::changeMinlength(int v){
    this->minlength = v;
    this->detect_lines();
    string windowName = "Frame preview";
    cv::imshow(windowName, this->imgL);
}

void CameraHandler::changePex(int v){
    this->error_p[0] = v/100;
    this->detect_lines();
    string windowName = "Frame preview";
    cv::imshow(windowName, this->imgL);
}
void CameraHandler::changePey(int v){
    this->error_p[1] = v/100;
    this->detect_lines();
    string windowName = "Frame preview";
    cv::imshow(windowName, this->imgL);
}

void CameraHandler::changeEw(int v){
    this->error_w = v/100;
    this->detect_lines();
    string windowName = "Frame preview";
    cv::imshow(windowName, this->imgL);
}

void CameraHandler::histogram(){
    this->stopline = false;
    cv::Mat img_gray, img_lines, mask;
    cvtColor(this->cv_image, img_gray, cv::COLOR_BGR2GRAY);
    int h = img_gray.rows, w = img_gray.cols;
    img_lines = this->cv_image.clone();
    mask = cv::Mat::zeros(img_gray.size(), CV_8U);
    cv::Point poly[4];
    poly[0] = cv::Point(0, 0.85 * h);
    poly[1] = cv::Point(w, 0.85 * h);
    poly[2] = cv::Point(w, h);
    poly[3] = cv::Point(0, h);
    const cv::Point *ppoly[1] = {poly};
    int npt[] = {4};
    fillPoly(mask, ppoly, npt, 1, 255, 8);
    bitwise_and(img_gray, mask, img_roi);
    cv::Mat thresh;
    threshold(img_roi, thresh, 150, 255, cv::THRESH_BINARY);
    cv::Mat hist = cv::Mat::zeros(1, w, CV_8U);
    for (int i = 0; i < w; i++){
        for (int j = 0; j < h; j++){
        if (thresh.at<uchar>(j, i) == 255)
            hist.at<uchar>(0, i) += 1;
        }
    }
    std::vector<int> lanes;
    int p = 0;
    for (int i = 0; i < w; i++){
        if (hist.at<uchar>(0, i) == 255 && p == 0){
        lanes.push_back(i);
        p = 255;
        }
        else if (hist.at<uchar>(0, i) == 0 && p == 255){
        lanes.push_back(i);
        p = 0;
        }
    }
    if (lanes.size() % 2 == 1)
        lanes.push_back(w - 1);
    std::vector<int> centers;
    for (int i = 0; i < lanes.size() / 2; i++){
        if (abs(lanes[2 * i] - lanes[2 * i + 1]) > 350)
        this->stopline = true;
        else if (abs(lanes[2 * i] - lanes[2 * i + 1]) > 3)
        centers.push_back((lanes[2 * i] + lanes[2 * i + 1]) / 2);
        line(img_lines, cv::Point((lanes[2 * i] + lanes[2 * i + 1]) / 2, 0.95 * h),
        cv::Point((lanes[2 * i] + lanes[2 * i + 1]) / 2, 0.85 * h),
        cv::Scalar(255, 0, 0), 3, 8);
    }
    int len_centers = centers.size();
    if (len_centers == 0) {
        lane_center = w / 2;
    } else if (len_centers == 1) {
        if (centers[0] > w / 2) {
            lane_center = (centers[0] + 0) / 2;
        } else {
            lane_center = (centers[0] + 600) / 2;
        }
    } else if (abs(centers[len_centers - 1] - centers[len_centers - 2]) < 200) {
        if ((centers[len_centers - 1] + centers[len_centers - 2]) > w) {
            lane_center = (centers[len_centers - 1] + centers[len_centers - 2] / 2 + 0) / 2;
        } else {
            lane_center = (centers[len_centers - 1] + centers[len_centers - 2] / 2 + 600) / 2;
        }
    } else {
        lane_center = (centers[len_centers - 1] + centers[len_centers - 2]) / 2;
    }
    cv::line(img_lines, cv::Point(int(lane_center), int(1 * h)), cv::Point(int(lane_center), int(0.8 * h)), cv::Scalar(255, 0, 255), 5);
    imgL = img_lines;
}