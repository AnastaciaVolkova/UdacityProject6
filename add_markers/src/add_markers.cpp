#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <sstream>
#include "add_markers/ManipMarker.h"

using namespace std;

ros::Publisher marker_pub;

// Handle for sevice /add_markers/ManipMarker.
bool handle_marker_manip(add_markers::ManipMarker::Request& req, add_markers::ManipMarker::Response& res){
    ros::Rate r(1);

    // Initialize marker.
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.258f;
    marker.color.g = 0.701f;
    marker.color.b = 0.921f;
    marker.color.a = 0.8;

    // Set action for marker according to request.
    if (req.to_add)
        marker.action = visualization_msgs::Marker::ADD;
    else
        marker.action = visualization_msgs::Marker::DELETE;

    // Set position for marker
    marker.pose.position.x = req.x;
    marker.pose.position.y = req.y;
    marker.pose.orientation.w = 1.0;

    // Print the message, which acknowledges request receiving.
    ROS_INFO("Receive request to %s marker at (%s, %s)", (req.to_add? "add" : "remove"), to_string(req.x).c_str(), to_string(req.y).c_str());

    marker.lifetime = ros::Duration();
    // Wait for visualization_marker topic subscribers
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
            return 0;
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    // Publish topic visualization_marker to rviz: show or delete marker.
    marker_pub.publish(marker);
    res.msg_feedback = "Done";
    return true;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/add_markers/ManipMarker", handle_marker_manip);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ROS_INFO("Ready to handle requests");
    ros::spin();
    return 0;
}
