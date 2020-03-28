#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // Build service call
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call DriveToTarget service!");
  }
  
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int target_color = 255;
    float rotation_rate = 1e-3;
    float linear_rate = 1e-3;

    // Number of pixels in a row to detect a ball
    int detection_theshold = 10;
    
    int peak_col_index = 0;
    int peak_count = 0;
    
    for (int col=0; col<img.width; col++){
      int col_count = 0;
      for (int row=0; row<img.height; row++) {
        if (img.data[col + row * img.step] == target_color) {
          col_count++;
        }
      }
      if (col_count > peak_count){
        peak_count = col_count;
        peak_col_index = col;
      }
    }
    
    float linear = 0.0;
    float rotation = 0.0;
    
    if (peak_count >= detection_theshold) {
      // the further away the ball is the faster we go
      linear = linear_rate/(float)peak_count;
      
      // the further from center the ball is the faster we rotate towards it
      rotation = rotation_rate/((float)img.width/2 - peak_col_index);
    }
    
    // Make a call to the drive service
    drive_robot(linear, rotation);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
