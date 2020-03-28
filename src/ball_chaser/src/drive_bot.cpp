#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <ball_chaser/DriveToTarget.h>

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// Service request handler
// Takes the requested linear and angular velocites and publishes them to the wheel joints.
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {

  geometry_msgs::Twist drive_cmd;
  drive_cmd.linear.x = req.linear_x;
  drive_cmd.angular.z = req.angular_z;
  motor_command_publisher.publish(drive_cmd);
  
  res.msg_feedback = "Setting drive to linear_x: " + std::to_string(req.linear_x);
  res.msg_feedback += " angular_z: " + std::to_string(req.angular_z);
  ROS_INFO_STREAM(res.msg_feedback);
  
  return true;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define DriveToTarget service
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
