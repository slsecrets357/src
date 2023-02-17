// #include "ros/ros.h"
// #include "message_filters/subscriber.h"
// #include "message_filters/time_synchronizer.h"
// #include <message_filters/synchronizer.h>
// #include "message_filters/sync_policies/approximate_time.h"
// #include "std_msgs/String.h"
// #include "utils/Lane.h"
// #include "utils/Sign.h"
// #include <cmath>
// #include <vector>

// using namespace message_filters;

// class LaneFollower {
// public:
//   std::vector<int> class_ids;
//   std::vector<int> boxes;
//   std::vector<float> scores;
//   std::vector<std::string> class_names = {"oneway", "highwayexit", "stopsign", "roundabout", "park", "crosswalk",
//                                           "noentry", "highwayentrance", "priority", "lights", "block", "pedestrian",
//                                           "car", "others", "nothing"};
//   std::vector<int> stop_sizes = {50, 50, 55, 50, 50, 65, 50, 50, 60, 25, 50, 100, 100};
//   int detected_id = 0;

//   float p = 0.006;
//   bool stopline = false;
//   std::string inter_dec = "straight";
//   float maxspeed = 0.15;
//   int i = 0;
//   float d = 0.003;
//   float last = 0;
//   int center = 0;
//   ros::Publisher cmd_vel_pub;
//   // ros::Rate rate;
//   static message_filters::Subscriber<utils::Lane> lane_sub;
//   static message_filters::Subscriber<utils::Sign> sign_sub;
//   double getSteeringAngle(double center);
//   void publishCmdVel(double steering_angle);
//   void callback(const utils::Lane::ConstPtr& lane, const utils::Sign::ConstPtr& sign);
// LaneFollower()
//   {
//     // ros::init(argc, argv, "lane_follower_node");
//     ros::NodeHandle nh;
//     cmd_vel_pub = nh.advertise<std_msgs::String>("/automobile/command", 1);
//     message_filters::Subscriber<utils::Lane> lane_sub(nh, "lane", 1);
//     message_filters::Subscriber<utils::Sign> sign_sub(nh, "sign", 1);
//     // lane_sub.subscribe(nh, "lane", 3);
//     // sign_sub.subscribe(nh, "sign", 3);
//     // message_filters::TimeSynchronizer<utils::Lane, utils::Sign> ts(lane_sub, sign_sub, 3, 0.15);
//     // ts.registerCallback(boost::bind(&LaneFollower::callback, this, _1, _2));
//   }
// };
// double LaneFollower::getSteeringAngle(double center) {
//     double image_center = 640 / 2;
//     double error = center - image_center;
//     double d_error = error - this->last;
//     this->last = error;
//     double steering_angle = error * this->p + d_error * this->d;
//     steering_angle = std::min(0.4, std::max(-0.4, steering_angle));
//     return steering_angle;
// }
// void LaneFollower::publishCmdVel(double steering_angle) {
//     std::string msgData;
//     double x = this->maxspeed + this->maxspeed * std::abs(steering_angle) / 0.4;
//     msgData = "{\"action\":\"1\",\"speed\":" + std::to_string(x) + "}";
//     this->cmd_vel_pub.publish(msgData);
//     msgData = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(steering_angle * 180 / M_PI) + "}";
//     this->cmd_vel_pub.publish(msgData);
// }
// void LaneFollower::callback(const utils::Lane::ConstPtr& lane, const utils::Sign::ConstPtr& sign) {
//     // Perform decision making tasks
//     // Compute the steering angle & linear velocity
//     // Publish the steering angle & linear velocity to the /automobile/command topic
//     center = lane->center;
//     std::vector<int> objects = sign->objects;
//     int numObj = sign->num;
//     std::vector<float> box1 = sign->box1;
//     std::vector<float> box2 = sign->box2;
//     for (int i = 0; i < numObj; i++) {
//     ROS_INFO("%s detected!", class_names[objects[i]].c_str());
//     }
//     // Determine the steering angle based on the center
//     double steering_angle = this->getSteeringAngle(center);
//     ROS_INFO("steering angle: %f", steering_angle);
//     // Publish the steering command
//     this->publishCmdVel(steering_angle);
// }


// int main(int argc, char** argv) {
//   ros::init(argc, argv, "lane_follower_node");
//   LaneFollower node;
//   typedef message_filters::sync_policies::ApproximateTime<utils::Lane, utils::Sign> MySyncPolicy;
//   message_filters::Synchronizer<MySyncPolicy> synchronizer(MySyncPolicy(100), LaneFollower::lane_sub, LaneFollower::sign_sub);
//   synchronizer.registerCallback(boost::bind(&LaneFollower::callback, _1, _2));
//   ros::Rate rate(10);
//   while (ros::ok()) {
//     rate.sleep();
//     ros::spinOnce();
//   }
//   return 0;
// }

