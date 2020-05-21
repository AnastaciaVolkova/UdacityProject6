#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "add_markers/ManipMarker.h"
#include <string>
#include <vector>
#include <sstream>

//Create typedef for a SimpleActionClient to communicate through MoveBaseAction interface.
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

// Structure to store goal parameters.
struct Goal {
    float x_, y_, w_; // Goal position
    string name_; // Goal name
    Goal(float x, float y, float w, string name):x_(x), y_(y), w_(w), name_(name){}
};


int main(int argc, char** argv){
  // Initialize the pick_objects node.
  ros::init(argc, argv, "pick_objects_node");
  ros::NodeHandle n;

  // Set service name to manipulate with marker through add_markers_node.
  string serv_name = "/add_markers/ManipMarker";

  // Define client to request service from add_markers_node.
  ros::ServiceClient client = n.serviceClient<add_markers::ManipMarker>(serv_name);

  // Wait for service availability.
  ros::service::waitForService(serv_name, -1);

  // Declare service to request from add_markers_node.
  add_markers::ManipMarker srv;

  //Construct a SimpleActionClient (SingleGoalActionClient) and spin up a thread to service this action's subscriptions.
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up.
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Declare and set up message for goal.
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define vector of goals (pickup, dropoff).
  vector<Goal> goals{Goal(-9.0, -5.0, 1.0, "pickup"), Goal(-4.0, 3.0, 1.0, "dropoff")};

  // Iteratare through goals.
  for (auto goal_it: goals){
    // Set request from add_markers_node.
    srv.request.x = goal_it.x_;
    srv.request.y = goal_it.y_;

    // Add a marker if goal is pickup.
    if (goal_it.name_ == "pickup"){
      srv.request.to_add = true;
      if (!client.call(srv))
          ROS_ERROR("Failed to call service %s", serv_name.c_str());
    }

    // Define a position and orientation for the robot to reach (define goal).
    goal.target_pose.pose.position.x = goal_it.x_;
    goal.target_pose.pose.position.y = goal_it.y_;
    goal.target_pose.pose.orientation.w = goal_it.w_;


    ROS_INFO("Sending %s goal: (%s, %s)", goal_it.name_.c_str(), to_string(goal_it.x_).c_str(), to_string(goal_it.y_).c_str());

    // Send the goal to acition client.
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("%s goal (%s, %s) is reached.", goal_it.name_.c_str(), to_string(goal_it.x_).c_str(), to_string(goal_it.y_).c_str());

      // Remove marker if it is on pickup place.
      // Add marker if it is on dropoff place.
      if (goal_it.name_ == "pickup")
        srv.request.to_add = false;
      else
        srv.request.to_add = true;
      if (!client.call(srv))
        ROS_ERROR("Failed to call service %s", serv_name.c_str());
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

    // Wait 5 seconds before moving away from pickup goal.
    if (goal_it.name_ == "pickup")
        ros::Duration(5).sleep();
  }
  return 0;
}

