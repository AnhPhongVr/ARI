// C++ standard headers
#include <sstream>

// ROS headers
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
 
  // Init the ROS node
  ros::init(argc, argv, "run_forward");
  ROS_INFO("Starting run_forward application ...");

  //Access point to communications with the ROS system
  ros::NodeHandle nodeHandler;

  // Create publisher
  ros::Publisher mobile_base_controller = nodeHandler.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);

  //Loops at a desired frequency in hz
  ros::Rate loop_rate(10);

  while(ros::ok()) {
    // Prepare data to send to topic
    geometry_msgs::Twist twist;

    geometry_msgs::Vector3 vector_linear;
    vector_linear.x = 0.5;
    vector_linear.y = 0.0;
    vector_linear.z = 0.0;

    geometry_msgs::Vector3 vector_angular;
    vector_angular.x = 0.0;
    vector_angular.y = 0.0;
    vector_angular.z = 0.0;

    twist.linear = vector_linear;
    twist.angular = vector_angular;

    // Publish data to the topic
    mobile_base_controller.publish(twist);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

// you are a wizard Harry