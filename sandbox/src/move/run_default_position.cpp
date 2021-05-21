// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>


// Our Action interface type for moving ARI's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;
typedef boost::shared_ptr< head_control_client>  head_control_client_Ptr;


// Create a ROS action client to move ARI's right arm
void createRightArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to right arm controller ...");

  actionClient.reset( new arm_control_client("/arm_right_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_right_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createRightArmClient: right arm controller action server not available");
}

void createLeftArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to left arm controller ...");

  actionClient.reset( new arm_control_client("/arm_left_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_left_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createLeftArmClient: left arm controller action server not available");
}

// Create a ROS action client to move ARI's right arm
void createHeadClient(head_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new arm_control_client("/head_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the head_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createHeadClient: head controller action server not available");
}


// Generates a simple trajectory with two waypoints to move ARI's arm 
void waypoints_right_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_right_1_joint");
  goal.trajectory.joint_names.push_back("arm_right_2_joint");
  goal.trajectory.joint_names.push_back("arm_right_3_joint");
  goal.trajectory.joint_names.push_back("arm_right_4_joint");


  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(4);
  goal.trajectory.points[index].positions[0] = 0;
  goal.trajectory.points[index].positions[1] = 0;
  goal.trajectory.points[index].positions[2] = 0;
  goal.trajectory.points[index].positions[3] = 0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(4);
  for (int j = 0; j < 4; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 3 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}

// Generates a simple trajectory with two waypoints to move ARI's arm 
void waypoints_left_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_left_1_joint"); // Joing at the shoulder, controls movement on the vertical axis
  goal.trajectory.joint_names.push_back("arm_left_2_joint"); // Joing at the shoulder, controls movement on the horizontal axis
  goal.trajectory.joint_names.push_back("arm_left_3_joint"); // Joing at the elbow, controls movement on the vertical axis
  goal.trajectory.joint_names.push_back("arm_left_4_joint"); // Joing at the elbow, controls movement on the horizontal axis


  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(4);
  goal.trajectory.points[index].positions[0] = 0;
  goal.trajectory.points[index].positions[1] = 0;
  goal.trajectory.points[index].positions[2] = 0;
  goal.trajectory.points[index].positions[3] = 0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(4);
  for (int j = 0; j < 4; ++j)
  {
    if (j != 0) {
      goal.trajectory.points[index].velocities[j] = 1.0;
    }
    else {
      goal.trajectory.points[index].velocities[j] = 0.5;
    }
  }
  // To be reached 3 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}

// Generates a simple trajectory with two waypoints to move ARI's arm 
void waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("head_1_joint");
  goal.trajectory.joint_names.push_back("head_2_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0;
  goal.trajectory.points[index].positions[1] = 0;

  // Velocities
  goal.trajectory.points[index].velocities.resize(2);
  for (int j = 0; j < 2; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 3 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
}


// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_dafault_position");

  ROS_INFO("Starting run_dab application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create an arm controller action client to move the ARI's right arm
  arm_control_client_Ptr RightArmClient;
  createRightArmClient(RightArmClient);
  arm_control_client_Ptr LeftArmClient;
  createLeftArmClient(LeftArmClient);
  head_control_client_Ptr HeadClient;
  createHeadClient(HeadClient);

  // Generates the goal for the ARI's right arm
  control_msgs::FollowJointTrajectoryGoal right_arm_goal;
  waypoints_right_arm_goal(right_arm_goal);
  control_msgs::FollowJointTrajectoryGoal left_arm_goal;
  waypoints_left_arm_goal(left_arm_goal);
  control_msgs::FollowJointTrajectoryGoal head_goal;
  waypoints_head_goal(head_goal);

  // Sends the command to start the given trajectory 1s from now
  right_arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  left_arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

  RightArmClient->sendGoal(right_arm_goal);
  LeftArmClient->sendGoal(left_arm_goal);
  HeadClient->sendGoal(head_goal);

  // Wait for trajectory execution
  while(!(RightArmClient->getState().isDone() && LeftArmClient->getState().isDone() && HeadClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(4).sleep(); // sleep for four seconds
  }

  return EXIT_SUCCESS;
}
