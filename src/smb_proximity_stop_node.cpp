#define pass (void)0

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

class ProximityStopNode {
public:
    ros::ServiceClient cli;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Subscriber sub;
    bool robotActive;
    ProximityStopNode(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void scanCallback(const sensor_msgs::LaserScan& msg);
}; 

ProximityStopNode::ProximityStopNode(ros::NodeHandle _nh, ros::NodeHandle _nh_private) {
    std::string sub_topic;
    int queue_size;
    nh = _nh;
    nh_private = _nh_private;
    nh_private.getParam("sub_topic", sub_topic);
    nh_private.getParam("queue_size", queue_size);
    robotActive = true;

    sub = nh.subscribe(sub_topic, queue_size, &ProximityStopNode::scanCallback, this); 
    cli = nh.serviceClient<std_srvs::SetBool>("toggleRobot");
}

void ProximityStopNode::scanCallback(const sensor_msgs::LaserScan& msg) {
    bool at_safe_distance = true;
    float safe_distance;
    nh_private.getParam("safe_distance", safe_distance);
    for (auto i : msg.ranges) i > safe_distance ? 1 : at_safe_distance = false;
    
    if (at_safe_distance == this->robotActive) return;
    
    ROS_INFO("Service called!");
    // Call the service to start / stop the robot.
    std_srvs::SetBool service;
    service.request.data = !this->robotActive;
    if (this->cli.call(service)) {
        this->robotActive = !this->robotActive;
        ROS_INFO("The robot has now %s", this->robotActive ? "started" : "stopped");
    } else {    
        ROS_ERROR("Failed to call service toggleRobot");
    }
    return;
}

int main (int argc, char** argv) {
	// ros::init must always be called before other functions
	ros::init(argc, argv, "smb_proximity_stop_node"); 
	// ros::NodeHandle is the API to the ROS system
    ros::NodeHandle nh, nh_private;
	nh = ros::NodeHandle();
	nh_private = ros::NodeHandle("~");
    ProximityStopNode ps(nh, nh_private);

	ros::spin();
}