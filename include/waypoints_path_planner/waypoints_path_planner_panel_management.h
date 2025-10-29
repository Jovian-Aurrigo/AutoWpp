/*
 * waypoints_path_planner_panel_management.h
 *
 *  Created on: 16 Mar 2024
 *      Author: maria-venus
 */

#ifndef WAYPOINTS_PATH_PLANNER_INCLUDE_WAYPOINTS_PATH_PLANNER_WAYPOINTS_PATH_PLANNER_PANEL_MANAGEMENT_H_
#define WAYPOINTS_PATH_PLANNER_INCLUDE_WAYPOINTS_PATH_PLANNER_WAYPOINTS_PATH_PLANNER_PANEL_MANAGEMENT_H_

#include <ros/ros.h>
#include<waypoints_path_planner/waypoints_path_planner.h>

using namespace std;

class waypoints_path_planner_panel_management
{

public:
	 waypoints_path_planner_panel_management();
	~waypoints_path_planner_panel_management();

	//waypoints_path_planner::WaypointsPathPlanner *ui;

	ros::NodeHandle nh_;

	/*Publishers*/
	/* Waypoints path planning localization management function declarations*/
	static void rviz_panel_locked();
	static void rviz_panel_unlocked();

};


#endif /* WAYPOINTS_PATH_PLANNER_INCLUDE_WAYPOINTS_PATH_PLANNER_WAYPOINTS_PATH_PLANNER_PANEL_MANAGEMENT_H_ */
