#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

void say_hello()
{
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::String>("/sound/say", 10);
	std_msgs::String msg;
	msg.data = "Bienvenue!";
	pub.publish(msg);

}

void wave()
{
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 10);
	trajectory_msgs::JointTrajectory msg;
	msg.joint_names = {"arm_right_1", "arm_right_2", "arm_right_3", "arm_right_4", "arm_right_5", "arm_right_6", "arm_right_7"};

	trajectory_msgs::JointTrajectoryPoint point1;
	point1.positions = {0.0, 0.5, 0.0, -1.5, 0.0, 0.0, 0.0};
	point1.time_from_start = ros::Duration(1.0);

	trajectory_msgs::JointTrajectoryPoint point2;
	point2.positions = {0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0};
	point2.time_from_start = ros::Duration(2.0);

	msg.points = {point1, point2};
	msg.header.stamp = ros::Time::now();

	pub.publish(msg);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "welcome_node");
	ros::NodeHandle nh;

	say_hello();
	wave();

	return 0;

}
