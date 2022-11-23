#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <cmath>
#include <smb_highlevel_controller/LaserScanner.hpp>

void LaserScanner::scanCallback(const sensor_msgs::LaserScan& msg) {
    float minRange = msg.range_max;
    float pillarAngle = 0;
    int n = 1;
    int detectorNo;

    // Get controller gain + angular tolerance (in rads)
    float linear_controller_gain, angular_controller_gain, angle_tolerance;
    nh_private.getParam("linear_controller_gain", linear_controller_gain);
    nh_private.getParam("angular_controller_gain", angular_controller_gain);
    nh_private.getParam("angle_tolerance", angle_tolerance);    

    // Angles in radians
    // ROS_INFO_STREAM("minAngle: " << msg.angle_min);
    // ROS_INFO_STREAM("angleIncrement: " << msg.angle_increment);
    // ROS_INFO_STREAM("maxAngle: " << msg.angle_max);
    
    for (auto const i : msg.ranges) {
        if (i < minRange) {
            minRange = i;
            detectorNo = n;
        }
        n++;
    };

    if (minRange == msg.range_max) ROS_INFO("No object detected.");
    else {
        pillarAngle = msg.angle_min + msg.angle_increment * detectorNo;
        ROS_INFO_STREAM("Angle of pillar: " << pillarAngle << std::endl);
        ROS_INFO_STREAM("Distance: " << minRange << std::endl);

        geometry_msgs::Twist smb_command;

        // Modify command
        if (pillarAngle > angle_tolerance) {
            smb_command.angular.z = angular_controller_gain;
        } else if (pillarAngle < angle_tolerance * -1) {
            smb_command.angular.z = angular_controller_gain * -1;
        } else {
            smb_command.linear.x = linear_controller_gain;
        }

        ROS_INFO_STREAM(smb_command << std::endl);
        this -> publish(smb_command);
        this -> tfListenerCallback(pillarAngle, minRange);
    }
};

void LaserScanner::publish(const geometry_msgs::Twist msg) {
    this -> pub.publish(msg);
}

void LaserScanner::tfListenerCallback(float pillarAngle, float minRange) {
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = this -> tfBuffer.lookupTransform("rslidar", "odom",
                                                            ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    auto translation = transformStamped.transform.translation;
    float detectedX = std::cos(minRange) * minRange;
    float detectedY = std::sin(minRange) * minRange;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = translation.x + detectedX;
    marker.pose.position.y = translation.y + detectedY;
    marker.pose.position.z = translation.z;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    this -> vis_marker_pub.publish(marker);
}

LaserScanner::LaserScanner(ros::NodeHandle _nh, ros::NodeHandle _nh_private) {    
    // Get the parameters from the server
    std::string sub_topic, pub_topic;
    int queue_size;
    nh = _nh;
    nh_private = _nh_private;
    nh_private.getParam("sub_topic", sub_topic);
    nh_private.getParam("pub_topic", pub_topic);
    nh_private.getParam("queue_size", queue_size);

    sub = nh.subscribe(sub_topic, queue_size, &LaserScanner::scanCallback, this);
    pub = nh.advertise<geometry_msgs::Twist>(pub_topic, 1);
    vis_marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    tfBuffer = tf2_ros::Buffer()
    tfListener = tf2_ros::TransformListener(tfBuffer);
};

int main(int argc, char *argv[]) {
    // Basic initalisation (boilerplate)
    ros::init(argc, argv, "smb_highlevel_controller");
    auto nh = ros::NodeHandle();
    auto nh_private = ros::NodeHandle("~");

    LaserScanner ls = LaserScanner(nh, nh_private);
    ros::spin();

    return 0;
}