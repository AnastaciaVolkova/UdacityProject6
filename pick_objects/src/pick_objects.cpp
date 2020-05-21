#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "add_markers/ManipMarker.h"
#include <string>
#include <vector>
#include <sstream>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

struct Goal {
    float x_, y_, w_; // Goal position
    string name_; // Goal name
    Goal(float x, float y, float w, string name):x_(x), y_(y), w_(w), name_(name){}
};


int main(int argc, char** argv){
  // Util variables for messages.
  stringstream message_iss;
  string message;

  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects_node");
  ros::NodeHandle n;
  string serv_name = "/add_markers/ManipMarker";

  ros::ServiceClient client = n.serviceClient<add_markers::ManipMarker>(serv_name);
  add_markers::ManipMarker srv;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define goals
  vector<Goal> goals{Goal(-9.0, -5.0, 1.0, "pickup"), Goal(-4.0, 3.0, 1.0, "dropoff")};
  for (auto goal_it: goals){
    srv.request.x = goal_it.x_;
    srv.request.y = goal_it.y_;
    if (goal_it.name_ == "pickup"){
      srv.request.to_add = true;
      ros::service::waitForService(serv_name, -1);
      if (!client.call(srv)){
          message_iss << "Failed to call service " << serv_name;
          message = message_iss.str();
          ROS_ERROR(message.c_str());
      } else {
          ROS_INFO(srv.response.msg_feedback.c_str());
      }
    }
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = goal_it.x_;
    goal.target_pose.pose.position.y = goal_it.y_;
    goal.target_pose.pose.orientation.w = goal_it.w_;

    // Send the goal position and orientation for the robot to reach
    message_iss << "Sending goal: (" << goal_it.x_ << ", " << goal_it.y_ << ")" << endl;
    message = message_iss.str();
    ROS_INFO(message.c_str());

    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      message_iss << goal_it.name_ << " goal (" << goal_it.x_ << ", " << goal_it.y_ << ") is reached.";
      message = message_iss.str();
      ROS_INFO(message.c_str());
      if (goal_it.name_ == "pickup")
        srv.request.to_add = false;
      else
        srv.request.to_add = true;
      if (!client.call(srv)){
        message_iss << "Failed to call service " << serv_name;
        message = message_iss.str();
        ROS_ERROR(message.c_str());
      } else
        ROS_INFO(srv.response.msg_feedback.c_str());
    } else {
          if (ac.getState()==actionlib::SimpleClientGoalState::StateEnum::PENDING)
              ROS_INFO("PENDING");
          else if(ac.getState()==actionlib::SimpleClientGoalState::StateEnum::ACTIVE)
              ROS_INFO("ACTIVE");
          else if(ac.getState()==actionlib::SimpleClientGoalState::StateEnum::RECALLED)
              ROS_INFO("RECALLED");
          else if(ac.getState()== actionlib::SimpleClientGoalState::StateEnum::REJECTED)
              ROS_INFO("REJECTED");
          else if(ac.getState()== actionlib::SimpleClientGoalState::StateEnum::PREEMPTED)
              ROS_INFO("PREEMPTED");
          else if(ac.getState()== actionlib::SimpleClientGoalState::StateEnum::ABORTED)
              ROS_INFO("ABORTED");
          else if(ac.getState()== actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
              ROS_INFO("SUCCEEDED");
          else if(ac.getState()== actionlib::SimpleClientGoalState::StateEnum::LOST)
              ROS_INFO("LOST");
    }
    ros::Duration(5).sleep();
  }
  return 0;
}

