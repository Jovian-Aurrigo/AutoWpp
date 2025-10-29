

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <vector>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <QApplication>

using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoints_path_planner");
	QApplication app(argc, argv);
	//file_management file;
	ROS_INFO("Starting Waypoints Path Planner");
	ros::spin();
}
