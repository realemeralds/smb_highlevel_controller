#ifndef LASERSCANNER_H
#define LASERSCANNER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>

class LaserScanner {
public:
    ros::Subscriber sub;
    ros::Publisher pub;
    bool active;
    ros::ServiceServer svr;
    ros::Publisher vis_marker_pub;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    LaserScanner(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void scanCallback(const sensor_msgs::LaserScan& msg); 
    void tfListenerCallback(float pillarAngle, float minRange);
    bool toggleRobot(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void publish(const geometry_msgs::Twist msg);
};

#endif // LASERSCANNER_H