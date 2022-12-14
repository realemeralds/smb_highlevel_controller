#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <smb_highlevel_controller/LaserScanner.hpp>
#include <string>

void LaserScanner::scanCallback(const sensor_msgs::LaserScan& msg)
{
  if (!this->active) {
    geometry_msgs::Twist smb_blank_command;
    this->publish(smb_blank_command);
    ROS_INFO_STREAM("Robot stopped, waiting for commands." << std::endl);
    return;
  }
  float minRange = msg.range_max;
  float pillarAngle = 0;
  int n = 1;
  int detectorNo = 0;

  // Get controller gain + angular tolerance (in rads)
  float linear_controller_gain, angular_controller_gain, angle_tolerance;
  nh_private.getParam("linear_controller_gain", linear_controller_gain);
  nh_private.getParam("angular_controller_gain", angular_controller_gain);
  nh_private.getParam("angle_tolerance", angle_tolerance);

  // Angles in radians
  // ROS_INFO_STREAM("minAngle: " << msg.angle_min);
  // ROS_INFO_STREAM("angleIncrement: " << msg.angle_increment);
  // ROS_INFO_STREAM("maxAngle: " << msg.angle_max);

  for (auto const i : msg.ranges)
  {
    if (i < minRange)
    {
      minRange = i;
      detectorNo = n;
    }
    n++;
  };

  if (minRange == msg.range_max)
    ROS_INFO("No object detected.");
  else
  {
    pillarAngle = msg.angle_min + msg.angle_increment * detectorNo;
    // ROS_INFO_STREAM("Angle of pillar: " << pillarAngle << std::endl);
    // ROS_INFO_STREAM("Distance: " << minRange << std::endl);

    geometry_msgs::Twist smb_command;

    // Modify command
    if (pillarAngle > angle_tolerance)
    {
      smb_command.angular.z = angular_controller_gain;
    }
    else if (pillarAngle < angle_tolerance * -1)
    {
      smb_command.angular.z = angular_controller_gain * -1;
    }
    else
    {
      smb_command.linear.x = linear_controller_gain;
    }

    // ROS_INFO_STREAM(smb_command << std::endl);
    this->publish(smb_command);
    this->tfListenerCallback(pillarAngle, minRange);
  }
};

void LaserScanner::publish(const geometry_msgs::Twist msg)
{
  this->pub.publish(msg);
}

void LaserScanner::tfListenerCallback(float pillarAngle, float minRange)
{
  // geometry_msgs::TransformStamped transformStamped;
  // try {
  //     transformStamped = tfBuffer.lookupTransform("odom", "rslidar",
  //     ros::Time(0));
  // }
  // catch (tf2::TransformException &ex) {
  //     ROS_WARN("%s",ex.what());
  //     ros::Duration(1.0).sleep();
  //     return;
  // }

  // ROS_INFO_STREAM("transform: " << transformStamped.transform << std::endl);

  geometry_msgs::PoseStamped lidar_pose;
  geometry_msgs::PoseStamped odom_pose;
  float detectedX = std::cos(pillarAngle) * minRange;
  float detectedY = std::sin(pillarAngle) * minRange;
  // ROS_INFO_STREAM("detectedX: " << detectedX << std::endl);
  // ROS_INFO_STREAM("detectedY: " << detectedY << std::endl);
  lidar_pose.header.frame_id = "rslidar";
  lidar_pose.header.stamp = ros::Time::now();
  lidar_pose.pose.position.x = detectedX;
  lidar_pose.pose.position.y = detectedY;
  lidar_pose.pose.position.z = 0;
  lidar_pose.pose.orientation.w = 1.0;
  lidar_pose.pose.orientation.x = 0.0;
  lidar_pose.pose.orientation.y = 0.0;
  lidar_pose.pose.orientation.z = 0.0;

  try
  {
    this->tfBuffer.transform<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>(lidar_pose, odom_pose, "odom",
                                                                                     ros::Duration(0.1));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  auto point_pos = odom_pose.pose.position;
  auto point_ori = odom_pose.pose.orientation;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = point_pos.x;
  marker.pose.position.y = point_pos.y;
  marker.pose.position.z = point_pos.z;
  marker.pose.orientation.x = point_ori.x;
  marker.pose.orientation.y = point_ori.y;
  marker.pose.orientation.z = point_ori.z;
  marker.pose.orientation.w = point_ori.w;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  this->vis_marker_pub.publish(marker);

  // Debug messages
  // ROS_INFO_STREAM("lidar ori: " << lidar_pose.pose.orientation << std::endl);
  // ROS_INFO_STREAM("pillarAngle: " << pillarAngle << std::endl);
  // ROS_INFO_STREAM("final ori:" << point_ori << std::endl);
}

bool LaserScanner::toggleRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  try
  {
    if (req.data)
    {
      this->active = true;
    }
    else
    {
      this->active = false;
    }
    res.success = true;
    res.message = this->active ? "Set robot state to true" : "Set robot state to false";
    return true;
  }
  catch (...)
  {
    return false;
  }
}

LaserScanner::LaserScanner(ros::NodeHandle _nh, ros::NodeHandle _nh_private) : tfListener(tfBuffer)
{
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
  vis_marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  active = true;
  svr = nh.advertiseService("toggleRobot", &LaserScanner::toggleRobot, this);
};

int main(int argc, char* argv[])
{
  // Basic initalisation (boilerplate)
  ros::init(argc, argv, "smb_highlevel_controller");
  auto nh = ros::NodeHandle();
  auto nh_private = ros::NodeHandle("~");
  LaserScanner ls(nh, nh_private);
  ros::Rate rate(10);

  // ros::spin();
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}