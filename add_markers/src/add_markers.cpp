#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include "add_markers/ManipMarker.h"

using namespace std;

ros::Publisher marker_pub;

bool handle_marker_manip(add_markers::ManipMarker::Request& req, add_markers::ManipMarker::Response& res){
    ros::Rate r(1);
    visualization_msgs::Marker marker;
    ROS_INFO("HANDLE of markder manip");
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    if (req.to_add)
        marker.action = visualization_msgs::Marker::ADD;
    else
        marker.action = visualization_msgs::Marker::DELETE;

    marker.pose.position.x = req.x;
    marker.pose.position.y = req.y; 
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.258f;
    marker.color.g = 0.701f;
    marker.color.b = 0.921f;
    marker.color.a = 0.8;
    
    marker.lifetime = ros::Duration();
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
        return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
    res.msg_feedback = "Done";
    return true;
};

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/add_markers/manip_marker", handle_marker_manip);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ROS_INFO("Ready to handle requests");
    ros::spin();
    return 0;
/*        
    

    marker_pub.publish(marker);
    ros::Duration(5).sleep();
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ros::Duration(5).sleep();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 3.5;
    marker.pose.orientation.w = 1.0;
    marker_pub.publish(marker);
    r.sleep();*/
}
