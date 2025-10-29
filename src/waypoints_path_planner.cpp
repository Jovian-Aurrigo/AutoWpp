/*
 *waypoints_path_planner.cpp
 *
 *  Created on: 29 Feb 2024
 *      Author: maria-venus
 */
#include <waypoints_path_planner/waypoints_path_planner.h>
using namespace std;
waypoints_path_planning_file_management wppfm;
waypoints_path_planning_waypoints_and_no_go_zones_managment wppngzm;
waypoints_path_planning_localization_management wpplm;
namespace waypoints_path_planner
{
WaypointsPathPlanner::WaypointsPathPlanner (QWidget * parent): rviz::Panel(parent),
		nh_("~"),
		does_main_lane_working_folder_exist_(false),
		does_left_lane_working_folder_exist_(false),
		does_right_lane_working_folder_exist_(false),
		does_temp_lane_working_folder_exist_(false),
		ifWorkingFileExist_(false),
		if_route_file_exist_(false),
		is_clicked(false),
		WorkingFolder_(""),
		line_number_of_vehicle_pose_in_the_global_plan_(0), //this initialisation needs testing
		is_filepath_received(false),
		is_goal_reached_(false),
		old_button_state_(0), //old_button_state_(16384)
		navigation_started_(false),
		check_localization_pose_(false),
		check_gps_pod_pose_(false),
		waypoints_size_(0),
		position_(0),
		no_go_zone_position_(0),
		does_route_exist_(false),
		boot_safe(false),
		previous_destination_string_(""),
		previous_origin_string_("")

		{
	ROS_INFO("WAYPOINTS PATH PLANNER!!!!");

	routes_manipulation_panel_=NULL;

	// Gets parameter from the launch file

	if (nh_.getParam("WorkingFolder", WorkingFolder_)) {
		boost::filesystem::path data_dir(WorkingFolder_);
		if (!boost::filesystem::is_directory(data_dir)) {
			ROS_ERROR("WorkingFolder_: '%s' does not exist!!", WorkingFolder_.data());
		} else {
			does_main_lane_working_folder_exist_ = true;
			does_left_lane_working_folder_exist_ = true;
			does_right_lane_working_folder_exist_ = true;
			does_temp_lane_working_folder_exist_ = false;
			ROS_INFO(" Param WorkingFolder_: '%s'", WorkingFolder_.data());
		}
	}
	if (nh_.getParam("WaypointsFilename", filename_)) {
		boost::filesystem::path data_dir(filename_);
		if (!boost::filesystem::is_regular_file(filename_)) {
			ROS_ERROR("filename_: '%s' does not exist!!", filename_.data());

		} else {
			ifWorkingFileExist_ = true;
			std::cout << "File Status " << ifWorkingFileExist_ << std::endl;
			ROS_INFO(" Segment_ filename_: '%s'", filename_.data());
		}

	}
	if (nh_.getParam("RoutesFilename", routes_filename_)) {
		boost::filesystem::path data_dir(routes_filename_);
		if (!boost::filesystem::is_regular_file(routes_filename_)) {
			ROS_ERROR("routes_file_name_: '%s' does not exist!!", routes_filename_.data());

		} else {
			if_route_file_exist_ = true;
			std::cout << "File Status " << if_route_file_exist_ << std::endl;
			ROS_INFO(" Routes_ filename_: '%s'", routes_filename_.data());
		}

	}
	if (nh_.getParam("WaypointsLeftLaneFilename", left_lane_filename_)) {
		boost::filesystem::path data_dir(left_lane_filename_);
		if (!boost::filesystem::is_regular_file(left_lane_filename_)) {
			ROS_ERROR("left_lane_filename_: '%s' does not exist!!", left_lane_filename_.data());

		} else {
			ifWorkingFileExist_ = true;
			std::cout << "File Status " << ifWorkingFileExist_ << std::endl;
			ROS_INFO(" Segment_ Left Lane filename_: '%s'", left_lane_filename_.data());
		}

	}
	if (nh_.getParam("WaypointsRightLaneFilename", right_lane_filename_)) {
		boost::filesystem::path data_dir(right_lane_filename_);
		if (!boost::filesystem::is_regular_file(right_lane_filename_)) {
			ROS_ERROR("right_lane_filename_: '%s' does not exist!!", right_lane_filename_.data());

		} else {
			ifWorkingFileExist_ = true;
			std::cout << "File Status " << ifWorkingFileExist_ << std::endl;
			ROS_INFO(" Segment_ Right Lane filename_: '%s'", right_lane_filename_.data());
		}

	}

	/*Subscribers*/
	nav_goal_to_plot_waypoints_subscriber_ = nh_.subscribe("/move_base_simple/goal", 10, & WaypointsPathPlanner::plot_waypoints_2d_nav_goal, this);
	point_clicked_from_rviz_to_plot_no_go_zones_subscriber_ = nh_.subscribe("/clicked_point", 10, & WaypointsPathPlanner::plot_no_go_zone_callback, this);
	dynamic_alignment_subscriber_ = nh_.subscribe("/align_waypoint", 1, & WaypointsPathPlanner::dynamic_alignment_callback, this);
	current_vehicle_position_subscriber_ = nh_.subscribe("/future_pod_pose_with_ln_in_global_plan", 10, & WaypointsPathPlanner::current_vehiclepose_from_localplanner_callback, this);
	global_plan_subscriber_ = nh_.subscribe("/car_like_global_planner_ros/detailed_route", 10, & WaypointsPathPlanner::detailed_route_from_globalplanner_callback, this);
	pause_set_subscriber_ = nh_.subscribe("/pause_set", 1, & WaypointsPathPlanner::pause_set_callback, this);
#ifdef VEH_TYPE_POD
	pause_release_subscriber_through_xbox_controller_ = nh_.subscribe(topic_prefix+"av_to_ads_controller", 10, & WaypointsPathPlanner::pause_release_from_controller_callback, this);
#endif
	position_error_subscriber_ = nh_.subscribe("/vehicle_position_error", 1, & WaypointsPathPlanner::vehicle_position_error_callback, this);
	is_local_goal_reached_subscriber_ = nh_.subscribe("/is_local_goal_reached", 1, & WaypointsPathPlanner::is_local_goal_reached_callback, this);
	is_av_navigating_subscriber_ = nh_.subscribe("/isPodNavigating", 10, & WaypointsPathPlanner::is_av_navigating_callback, this); // change topic name
	localization_pose_subscriber_ = nh_.subscribe("/pose/fused", 10, & WaypointsPathPlanner::localization_pose_callback, this);
	gps_pose_subscriber_ = nh_.subscribe("/odometry/filtered", 10, & WaypointsPathPlanner::gps_pod_pose_callback, this);
	stop_requested_subscriber_ = nh_.subscribe("/stop_requested", 1, & WaypointsPathPlanner::av_stop_request_callback, this);
	dms_handle_on_off_subscriber_ = nh_.subscribe("/DM_Speed_Reduction_Factor_Message", 10, & WaypointsPathPlanner::dms_status_callback, this);
	microstrain_imu_subscriber_ = nh_.subscribe("/gx5/imu/data", 1000, & WaypointsPathPlanner::micro_imu_callback, this);
	merged_point_cloud_subscriber_ = nh_.subscribe("/points_concat", 100, & WaypointsPathPlanner::merged_cloud_callback, this);
	time_machine_subscriber_ = nh_.subscribe("/Timemachine_Info", 10, & WaypointsPathPlanner::time_machine_callback, this);
	system_monitor_subscriber_ = nh_.subscribe("/aurrigo_System_Monitor", 10, & WaypointsPathPlanner::system_monitor_callback, this);
	bbr_recorder_subscriber_ = nh_.subscribe("/BlackBox_Info", 10, & WaypointsPathPlanner::black_box_recorder_callback, this);
	pod_to_acs_core_subscriber_ = nh_.subscribe(topic_prefix+"av_to_ads_core", 10, & WaypointsPathPlanner::pod_to_acs_core_callback, this);
	pod_to_acs_aux_subscriber_ = nh_.subscribe(topic_prefix+"av_to_ads_aux", 10, & WaypointsPathPlanner::av_to_ads_aux_callback, this);
	current_waypoint_index_subscriber_ = nh_.subscribe("/current_waypoint_index", 10, &WaypointsPathPlanner::current_waypoint_index_callback, this);
	global_map_path_subscriber_ = nh_.subscribe("/map_file_path",10,&WaypointsPathPlanner::global_map_path_callback, this);
	end_of_route_subscriber_ = nh_.subscribe("/end_of_route", 1, &WaypointsPathPlanner::end_of_route_subscriber_callback, this);
	local_goal_reached_subscriber_=nh_.subscribe("/goal_reached",1,&WaypointsPathPlanner::local_goal_reached_subscriber_callback, this);
	ipe_to_ads_subscriber_ = nh_.subscribe("/av_to_ads_trip_request", 10, & WaypointsPathPlanner::ipe_to_ads_callback, this);
	lane_switch_subscriber_ = nh_.subscribe("/lane_switch", 10, & WaypointsPathPlanner::lane_switch_callback, this);
	waypointinfo_subscriber_ = nh_.subscribe("/waypoint_info", 1, &WaypointsPathPlanner::waypoint_info_callback, this);
	minder_state_subscriber_ = nh_.subscribe("/asm/minder_state", 1, &WaypointsPathPlanner::minder_state_callback, this);

	/*Service Client*/
	start_navigation_service_ = nh_.serviceClient < waypoints_path_planner::startNavigation > ("/start_navigation");
	cancel_navigation_service_ = nh_.serviceClient < std_srvs::Trigger > ("/cancel_navigation");

	/*Publishers*/
	safety_supervisor_name_publisher_ = nh_.advertise < std_msgs::String > ("/safety_supervisor_name", 1, true);
	lock_unlock_rviz_status_publisher_ = nh_.advertise < std_msgs::Bool > ("/isrvizeditlocked", 1, true);
	pause_release_publisher_ = nh_.advertise < std_msgs::Int32 > ("/release_pause", 1, true);
	map_file_path_publisher_ = nh_.advertise < std_msgs::String > ("/map_file_path", 1, true);
	ads_to_ipe_publisher_ = nh_.advertise < std_msgs::String> ("/ads_to_av_trip_status", 1, true);
	file_path_publisher_ = nh_.advertise <waypoints_path_planner::waypointsfilepath>("/waypoints_file_path", 1, true);



	/*Timers*/
	timer_for_vehicle_position_error_status_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_vehicle_position_error, this));
	timer_for_vehicle_position_error_status_.start();
	timer_for_navigation_status_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_navigation_callback, this));
	timer_for_navigation_status_.start();
	timer_for_battery_status_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_pod_to_acs_core_callback, this));
	timer_for_battery_status_.start();
	timer_for_dms_handle_status_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_dms_handle_callback, this));
	timer_for_dms_handle_status_.start();
	//timer_for_gps_fix_on_off_=nh.createTimer(ros::Duration(1), boost::bind (&InteractiveWaypoints::timerforGPSfixCallback,this));
	//timer_for_gps_fix_on_off_.start();
	timer_for_imu_fix_on_off_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_imu_callback, this));
	timer_for_imu_fix_on_off_.start();
	timer_for_bbr_status_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_bbr_callback, this));
	timer_for_bbr_status_.start();
	timer_for_time_machine_status_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_time_machine_callback, this));
	timer_for_time_machine_status_.start();
	timer_for_system_monitor_status_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_system_monitor_callback, this));
	timer_for_system_monitor_status_.start();
	timer_for_loading_arms_status_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_loading_arms_callback, this));
	timer_for_loading_arms_status_.start();
	timer_for_lidar_status_on_off_ = nh_.createTimer(ros::Duration(1), boost::bind( & WaypointsPathPlanner::timer_for_lidar_status_callback, this));
	timer_for_lidar_status_on_off_.start();

	/* get parameters via the nodehandle and override the default config of car planner*/

	cfg_.loadRosParamFromNodeHandle(nh_);
	// create visualization instance
	Visualizer_ = boost::make_shared < carlike_local_planner::CarLikeVisualization > ();
	Visualizer_ -> initialize(nh_, cfg_);
	Planner_ = boost::make_shared < CarLocalPlanner > (Visualizer_, boost::ref(cfg_));

	/* Waypoints path planner panel layout using qt */

	QGridLayout * gridLayout = new QGridLayout;

	/* Panel Title*/

	QLabel * title1 = new QLabel("AURRIGO");
	title1 -> setAlignment(Qt::AlignRight);
	title1 -> setFont(QFont("FreeSans", 72));
	title1 -> setStyleSheet("font-weight: bold;color: black; font-size:12pt");
	//title1->setWordWrap(true);
	gridLayout -> addWidget(title1, 0, 0, 1, 1);

	/* Panel Sub Title*/

	QLabel * title2 = new QLabel("AUTO-STACK");
	title2 -> setAlignment(Qt::AlignLeft);
	title2 -> setFont(QFont("FreeSans", 72));
	title2 -> setStyleSheet("font-weight: bold;color: black; font-size:12pt");
	//title2->setWordWrap(true);
	gridLayout -> addWidget(title2, 0, 1, 1, 1);

	/* Navigation Label*/

	QLabel * label1 = new QLabel("NAVIGATION");
	label1 -> setAlignment(Qt::AlignLeft);
	label1 -> setFont(QFont("FreeSans", 72));
	label1 -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
	label1 -> setWordWrap(true);
	gridLayout -> addWidget(label1, 1, 0, 1, 1);

	/* Button to pose estimate AV */

	vehicle_pose_estimate_ = new QPushButton("POSE-ESTIMATE");
	gridLayout -> addWidget(vehicle_pose_estimate_, 2, 0, 1, 1);
	vehicle_pose_estimate_ -> setEnabled(true);
	vehicle_pose_estimate_ -> setFixedSize(150, 30);
	vehicle_pose_estimate_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	vehicle_pose_estimate_ -> setFont(QFont("FreeSans"));
	connect(vehicle_pose_estimate_, SIGNAL(released()), this, SLOT(set_poseestimate_qslot()));

	/* Button to change map */

	change_map_ = new QPushButton("LOAD NEW MAP");
	gridLayout -> addWidget(change_map_, 5, 3, 1, 1);
	change_map_ -> setEnabled(true);
	change_map_ -> setFixedSize(150, 30);
	change_map_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	change_map_ -> setFont(QFont("FreeSans"));
	connect(change_map_, SIGNAL(released()), this, SLOT(load_newmap_qslot()));

	/* Button to start AV navigation */

	start_navigation_ = new QPushButton("START NAVIGATION", this);
	gridLayout -> addWidget(start_navigation_, 2, 2, 1, 1);
	start_navigation_ -> setEnabled(true);
	start_navigation_ -> setFixedSize(150, 30);
	start_navigation_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	start_navigation_ -> setFont(QFont("FreeSans"));
	connect(start_navigation_, SIGNAL(released()), this, SLOT(start_av_navigation_qslot()));

	/* Button to Cancel AV navigation */

	cancel_navigation_ = new QPushButton("CANCEL NAVIGATION", this);
	gridLayout -> addWidget(cancel_navigation_, 2, 3, 1, 1);
	cancel_navigation_ -> setEnabled(true);
	cancel_navigation_ -> setFixedSize(150, 30);
	cancel_navigation_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	cancel_navigation_ -> setFont(QFont("FreeSans"));
	connect(cancel_navigation_, SIGNAL(released()), this, SLOT(cancel_navigation_qslot()));

	/* Panel Sub title */

	QLabel * rndf_label_ = new QLabel("RNDF");
	rndf_label_ -> setAlignment(Qt::AlignLeft);
	rndf_label_-> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
	rndf_label_ -> setWordWrap(true);
	rndf_label_ -> setFont(QFont("FreeSans", 72));
	gridLayout -> addWidget(rndf_label_, 3, 0, 1, 1);

	/* Button to lock/unlock rviz functionality*/

	toggle_lock_unlock_ = new QPushButton;
	gridLayout -> addWidget(toggle_lock_unlock_, 3, 1, 1, 1);

	nh_.getParam("/toggle_button_lock_file", toggle_button_lock_file_);
	qtoggle_button_lock_file_ = QString::fromStdString(toggle_button_lock_file_.data());
	QPixmap lock_lock_pixmap(qtoggle_button_lock_file_);
	nh_.getParam("/toggle_button_unlock_file", toggle_button_unlock_file_);
	qtoggle_button_unlock_file_ = QString::fromStdString(toggle_button_unlock_file_.data());
	QPixmap lock_unlock_pixmap(qtoggle_button_unlock_file_);
	toggle_lock_unlock_ -> setIcon(QIcon(qtoggle_button_lock_file_));
	toggle_lock_unlock_ -> setIconSize(lock_lock_pixmap.rect().size());
	toggle_lock_unlock_ -> setFixedSize(lock_lock_pixmap.rect().size());
	toggle_lock_unlock_ -> setEnabled(true);
	toggle_lock_unlock_ -> setFixedSize(150, 50);
	toggle_lock_unlock_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	toggle_lock_unlock_ -> setFont(QFont("FreeSans"));
	toggle_lock_unlock_ -> setCheckable(true);
	toggle_lock_unlock_ -> setChecked(false);
	connect(toggle_lock_unlock_, SIGNAL(released()), this, SLOT(lock_unlock_rvizedit_qslot()));

	/* Button to switch to left lane */

	left_lane_switch_ = new QPushButton("MERGE LEFT");
	gridLayout -> addWidget(left_lane_switch_, 3, 2, 1, 1);
	left_lane_switch_ -> setEnabled(true);
	left_lane_switch_ -> setFixedSize(150, 30);
	left_lane_switch_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	left_lane_switch_ -> setFont(QFont("FreeSans"));
	connect(left_lane_switch_, SIGNAL(clicked()), this, SLOT(switch_to_leftlane_qslot()));

	/* Button to switch to Right Lane */

	right_lane_switch_ = new QPushButton("MERGE RIGHT");
	gridLayout -> addWidget(right_lane_switch_, 3, 3, 1, 1);
	right_lane_switch_ -> setEnabled(true);
	right_lane_switch_ -> setFixedSize(150, 30);
	right_lane_switch_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	right_lane_switch_ -> setFont(QFont("FreeSans"));
	connect(right_lane_switch_, SIGNAL(clicked()), this, SLOT(switch_to_rightlane_qslot()));

	/* Button to create new waypoints file */

	create_waypoint_file_ = new QPushButton("CREATE SEGMENT FILE", this);
	gridLayout -> addWidget(create_waypoint_file_, 4, 0, 1, 1);
	create_waypoint_file_ -> setEnabled(true);
	create_waypoint_file_ -> setFixedSize(150, 30);
	create_waypoint_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	create_waypoint_file_ -> setFont(QFont("FreeSans"));
	connect(create_waypoint_file_, SIGNAL(clicked()), this, SLOT(create_waypoint_file_qslot()));

	/* Button to load waypoints file */

	load_waypoints_file_ = new QPushButton("LOAD SEGMENT", this);
	gridLayout -> addWidget(load_waypoints_file_, 4, 1, 1, 1);
	load_waypoints_file_ -> setEnabled(true);
	load_waypoints_file_ -> setFixedSize(150, 30);
	load_waypoints_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	load_waypoints_file_ -> setFont(QFont("FreeSans"));
	connect(load_waypoints_file_, SIGNAL(released()), this, SLOT(load_waypoints_file_qslot()));

	/* Button to clear all waypoints markers on rviz */

	clear_waypoint_markers_ = new QPushButton("CLEAR WAYPOINTS", this);
	gridLayout -> addWidget(clear_waypoint_markers_, 4, 2, 1, 1);
	clear_waypoint_markers_ -> setEnabled(true);
	clear_waypoint_markers_ -> setFixedSize(150, 30);
	clear_waypoint_markers_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	clear_waypoint_markers_ -> setFont(QFont("FreeSans"));
	connect(clear_waypoint_markers_, SIGNAL(clicked()), this, SLOT(clearall_waypoint_markers_qslot()));

	/* Button to place waypoints using localised vehicle pose */

	place_lidar_waypoints_ = new QPushButton("PLACE LIDAR WAYP'T", this);
	gridLayout -> addWidget(place_lidar_waypoints_, 5, 0, 1, 1);
	place_lidar_waypoints_ -> setEnabled(true);
	place_lidar_waypoints_ -> setFixedSize(150, 30);
	place_lidar_waypoints_ -> setFont(QFont("FreeSans"));
	place_lidar_waypoints_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(place_lidar_waypoints_, SIGNAL(clicked()), this, SLOT(place_lidar_waypoints_qslot()));

	/* Button to place waypoints based on GPS RTK fix */

	place_gps_waypoints_ = new QPushButton("PLACE GPS WAYP'T", this);
	gridLayout -> addWidget(place_gps_waypoints_, 5, 1, 1, 1);
	place_gps_waypoints_ -> setEnabled(true);
	place_gps_waypoints_ -> setFixedSize(150, 30);
	place_gps_waypoints_ -> setFont(QFont("FreeSans"));
	place_gps_waypoints_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(place_gps_waypoints_, SIGNAL(clicked()), this, SLOT(place_gps_waypoints_qslot()));

	/* Button to compute trajectory between waypoints */

	compute_segment_ = new QPushButton("COMPUTE SEGMENT", this);
	gridLayout -> addWidget(compute_segment_, 5, 2, 1, 1);
	compute_segment_ -> setEnabled(true);
	compute_segment_ -> setFixedSize(150, 30);
	compute_segment_ -> setFont(QFont("FreeSans"));
	compute_segment_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(compute_segment_, SIGNAL(clicked()), this, SLOT(compute_segment_qslot()));

	/* Button to release pause in AV*/

	pause_release_ = new QPushButton("PAUSE RELEASE", this);
	gridLayout -> addWidget(pause_release_, 2, 1, 1, 1);
	pause_release_ -> setEnabled(true);
	pause_release_ -> setFixedSize(150, 30);
	pause_release_ -> setFont(QFont("FreeSans"));
	pause_release_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(pause_release_, SIGNAL(released()), this, SLOT(pause_release_qslot()));

	/* Route label */

	QLabel * routes_title = new QLabel(" ROUTING ");
	routes_title -> setAlignment(Qt::AlignLeft);
	routes_title -> setFont(QFont("FreeSans", 72));
	routes_title -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
	//title->setWordWrap(true);
	gridLayout -> addWidget(routes_title, 15, 0, 1, 1);

	/* Button to create a route file */

	create_route_file_= new QPushButton("CREATE ROUTE FILE", this);
	gridLayout -> addWidget(create_route_file_, 15, 1, 1, 1);
	create_route_file_ -> setEnabled(true);
	create_route_file_ -> setFixedSize(150, 30);
	create_route_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	create_route_file_ -> setFont(QFont("FreeSans"));
	connect(create_route_file_, SIGNAL(clicked()), this, SLOT(create_route_file_qslot()));

	/* Button to load a route file */

	load_route_file_= new QPushButton("LOAD ROUTE FILE", this);
	gridLayout -> addWidget(load_route_file_, 15, 2, 1, 1);
	load_route_file_ -> setEnabled(true);
	load_route_file_ -> setFixedSize(150, 30);
	load_route_file_ -> setFont(QFont("FreeSans"));
	load_route_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(load_route_file_, SIGNAL(released()), this, SLOT(load_route_file_qslot()));

	/* Button to set a route file */

	set_route_file_= new QPushButton("SET ROUTE FILE", this);
	gridLayout -> addWidget(set_route_file_, 15, 3, 1, 1);
	set_route_file_ -> setEnabled(true);
	set_route_file_ -> setFixedSize(150, 30);
	set_route_file_ -> setFont(QFont("FreeSans"));
	set_route_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(set_route_file_, SIGNAL(released()), this, SLOT(set_route_file_qslot()));

	/* Button to run route 1 */

	run_route_1_file_= new QPushButton("RUN SEGMENT 1", this);
	gridLayout -> addWidget(run_route_1_file_, 16, 0, 1, 1);
	run_route_1_file_ -> setEnabled(true);
	run_route_1_file_ -> setFixedSize(150, 30);
	run_route_1_file_ -> setFont(QFont("FreeSans"));
	run_route_1_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(run_route_1_file_, SIGNAL(released()), this, SLOT(run_route_1_qslot()));

	/* Button to run route 2 */

	run_route_2_file_= new QPushButton("RUN SEGMENT 2", this);
	gridLayout -> addWidget(run_route_2_file_, 16, 1, 1, 1);
	run_route_2_file_ -> setEnabled(true);
	run_route_2_file_ -> setFixedSize(150, 30);
	run_route_2_file_ -> setFont(QFont("FreeSans"));
	run_route_2_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(run_route_2_file_, SIGNAL(released()), this, SLOT(run_route_2_qslot()));

	/* Button to run route 3 */

	run_route_3_file_= new QPushButton("RUN SEGMENT 3", this);
	gridLayout -> addWidget(run_route_3_file_, 16, 2, 1, 1);
	run_route_3_file_ -> setEnabled(true);
	run_route_3_file_ -> setFixedSize(150, 30);
	run_route_3_file_ -> setFont(QFont("FreeSans"));
	run_route_3_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(run_route_3_file_, SIGNAL(released()), this, SLOT(run_route_3_qslot()));

	/* Button to run route 4 */

	run_route_4_file_= new QPushButton("RUN SEGMENT 4", this);
	gridLayout -> addWidget(run_route_4_file_, 16, 3, 1, 1);
	run_route_4_file_ -> setEnabled(true);
	run_route_4_file_ -> setFixedSize(150, 30);
	run_route_4_file_ -> setFont(QFont("FreeSans"));
	run_route_4_file_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	connect(run_route_4_file_, SIGNAL(released()), this, SLOT(run_route_4_qslot()));

	/*Button's to delete no go zone markers on rviz and file */

	delete_no_go_zone_markers_in_file_and_rviz_ = new QPushButton("CLEAR TD ZONES");
	gridLayout -> addWidget(delete_no_go_zone_markers_in_file_and_rviz_, 4, 3, 1, 1);
	delete_no_go_zone_markers_in_file_and_rviz_ -> setEnabled(true);
	delete_no_go_zone_markers_in_file_and_rviz_ -> setFixedSize(150, 30);
	delete_no_go_zone_markers_in_file_and_rviz_ -> setFont(QFont("FreeSans"));
	delete_no_go_zone_markers_in_file_and_rviz_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
	//set to true when the no go zone functionality is fully developed
	delete_no_go_zone_markers_in_file_and_rviz_ -> setVisible(true);
	connect(delete_no_go_zone_markers_in_file_and_rviz_, SIGNAL(released()), this, SLOT(delete_no_go_zone_markers_in_file_and_rviz_qslot()));

	/* Status title 1 label */

	QLabel * status_title1 = new QLabel(" SYSTEM ");
	status_title1 -> setAlignment(Qt::AlignRight);
	status_title1 -> setFont(QFont("FreeSans", 72));
	status_title1 -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
	//title->setWordWrap(true);
	gridLayout -> addWidget(status_title1, 17, 1, 1, 1);

	/* Status title 2 label */

	QLabel * status_title2 = new QLabel("STATUS");
	status_title2 -> setAlignment(Qt::AlignLeft);
	status_title2 -> setFont(QFont("FreeSans", 72));
	status_title2 -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
	//title->setWordWrap(true);
	gridLayout -> addWidget(status_title2, 17, 2, 1, 1);

	/* Battery status label */

	label_battery_default_ = new QLabel;
	label_battery_default_ -> setAlignment(Qt::AlignLeft);
	label_battery_default_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_battery_default_ -> setWordWrap(true);
	label_battery_default_ -> setFont(QFont("FreeSans", 72));
	label_battery_default_ -> setVisible(true);
	gridLayout -> addWidget(label_battery_default_, 18, 0, 1, 1);
	nh_.getParam("/battery_unknown_file", battery_unknown_file_);
	qbattery_unknown_file_ = battery_unknown_file_.data();
	movie_battery_unknown_ = new QMovie(qbattery_unknown_file_);
	//movie_battery_unknown__->setScaledSize(QSize(80,50));
	label_battery_default_ -> setMovie(movie_battery_unknown_);
	//label_battery_default_->setScaledContents( true );
	//label_battery_default_->setSizePolicy( QSizePolicy::Preferred, QSizePolicy::Preferred );
	movie_battery_unknown_ -> start();
	nh_.getParam("/battery_charging_file", battery_charging_file_);
	qbattery_charging_file_=battery_charging_file_.data();
	movie_battery_charging_ = new QMovie(qbattery_charging_file_);
	nh_.getParam("/battery_depleted_file", battery_depleted_file_);
	qbattery_depleted_file_=battery_depleted_file_.data();
	movie_battery_depleated_ = new QMovie(qbattery_depleted_file_);
	nh_.getParam("/battery_full_file",battery_full_file_);
	qbattery_full_file_=battery_full_file_.data();
	movie_battery_full_ = new QMovie(qbattery_full_file_);
	nh_.getParam("/battery_half_file", battery_half_file);
	qbattery_half_file=battery_half_file.data();
	movie_battery_half_ = new QMovie(qbattery_half_file);
	nh_.getParam("/battery_quarter_file", battery_quarter_file_);
	qbattery_quarter_file_=battery_quarter_file_.data();
	movie_battery_quarter_ = new QMovie(qbattery_quarter_file_);
	nh_.getParam("/battery_three_quarter_file", battery_three_quarter_file_);
	qbattery_three_quarter_file_=battery_three_quarter_file_.data();
	movie_battery_three_quarter_ = new QMovie(qbattery_three_quarter_file_);
	nh_.getParam("/battery_charger_plugged_in_file", battery_charger_plugged_in_file_);
	qbattery_charger_plugged_in_file_=battery_charger_plugged_in_file_.data();
	movie_battery_full_plugged_in_ = new QMovie(qbattery_charger_plugged_in_file_);

	/* Autonomy status label */

	label_autonomy_enable_disable_ = new QLabel;
	label_autonomy_enable_disable_ -> setAlignment(Qt::AlignLeft);
	label_autonomy_enable_disable_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_autonomy_enable_disable_ -> setWordWrap(true);
	label_autonomy_enable_disable_ -> setFont(QFont("FreeSans", 72));
	label_autonomy_enable_disable_ -> setVisible(true);
	gridLayout -> addWidget(label_autonomy_enable_disable_, 18, 1, 1, 1);
	nh_.getParam("/autonomy_switch_unknown_file", autonomy_switch_unknown_file_);
	qautonomy_switch_unknown_file_=autonomy_switch_unknown_file_.data();
	movie_autonomy_enable_on_off_unknown_ = new QMovie(qautonomy_switch_unknown_file_);
	label_autonomy_enable_disable_ -> setMovie(movie_autonomy_enable_on_off_unknown_);
	movie_autonomy_enable_on_off_unknown_ -> start();
	nh_.getParam("/autonomy_switch_on_file", autonomy_switch_on_file_);
	qautonomy_switch_on_file_=autonomy_switch_on_file_.data();
	movie_autonomy_enable_on_ = new QMovie(qautonomy_switch_on_file_);
	nh_.getParam("/autonomy_switch_off_file", autonomy_switch_off_file_);
	qautonomy_switch_off_file_=autonomy_switch_off_file_.data();
	movie_autonomy_enable_off_ = new QMovie(qautonomy_switch_off_file_);

	/* Dms on/off status label */

	label_dms_on_off_ = new QLabel;
	label_dms_on_off_ -> setAlignment(Qt::AlignLeft);
	label_dms_on_off_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_dms_on_off_ -> setWordWrap(true);
	label_dms_on_off_ -> setFont(QFont("FreeSans", 72));
	label_dms_on_off_ -> setVisible(true);
	gridLayout -> addWidget(label_dms_on_off_, 18, 2, 1, 1);
	nh_.getParam("/dms_unknown_file", dms_unknown_file_);
	qdms_unknown_file_=dms_unknown_file_.data();
	movie_dms_on_off_unknown_ = new QMovie(qdms_unknown_file_);
	label_dms_on_off_ -> setMovie(movie_dms_on_off_unknown_);
	movie_dms_on_off_unknown_ -> start();
	nh_.getParam("/dms_not_initialised_file",dms_not_initialised_file_);
	qdms_not_initialised_file_=dms_not_initialised_file_.data();
	movie_dms_on_off_uninitialised_ = new QMovie(qdms_not_initialised_file_);
	nh_.getParam("/dms_not_armed_file",dms_not_armed_file_);
	qdms_not_armed_file_=dms_not_armed_file_.data();
	movie_dms_off_ = new QMovie(qdms_not_armed_file_);
	nh_.getParam("/dms_armed_file", dms_armed_file_);
	qdms_armed_file_=dms_armed_file_.data();
	movie_dms_on_ = new QMovie(qdms_armed_file_);
	//movie_battery_removed_=new QMovie("/home/aurrigo-ads/aurrigo_3d_navigation_20.04_ws/src/interactive_waypoints/resources/DMS/DMS Armed.gif");

	/* Dtp on/off status label */

	bool s;
	nh_.getParam("/vehicle_communication/ACS_endpoint_safety_enabled", s);

	label_dtp_on_off_ = new QLabel;
	label_dtp_on_off_ -> setAlignment(Qt::AlignLeft);
	label_dtp_on_off_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_dtp_on_off_ -> setWordWrap(true);
	label_dtp_on_off_ -> setFont(QFont("FreeSans", 72));
	label_dtp_on_off_ -> setVisible(true);
	gridLayout -> addWidget(label_dtp_on_off_, 18, 3, 1, 1);
	nh_.getParam("/dtp_unknown_file", dtp_unknown_file_);
	qdtp_unknown_file_=dtp_unknown_file_.data();
	movie_dtp_on_off_unknown_ = new QMovie(qdtp_unknown_file_);
	label_dtp_on_off_ -> setMovie(movie_dtp_on_off_unknown_);
	movie_dtp_on_off_unknown_ -> start();
	nh_.getParam("/dtp_on_file", dtp_on_file_);
	qdtp_on_file_= dtp_on_file_.data();
	movie_dtp_on_ = new QMovie(qdtp_on_file_);
	nh_.getParam("/dtp_off_file", dtp_off_file_);
	qdtp_off_file_=dtp_off_file_.data();
	movie_dtp_off_ = new QMovie(qdtp_off_file_);
	if (s == true) {
		label_dtp_on_off_ -> setMovie(movie_dtp_on_);
		movie_dtp_on_ -> start();
	} else if (s == false) {
		label_dtp_on_off_ -> setMovie(movie_dtp_off_);
		movie_dtp_off_ -> start();
	} else {
		label_dtp_on_off_ -> setMovie(movie_dtp_on_off_unknown_);
		movie_dtp_on_off_unknown_ -> start();
	}

	/* Imu fix on/off status label */

	label_imu_fix_on_off_ = new QLabel;
	label_imu_fix_on_off_ -> setAlignment(Qt::AlignLeft);
	label_imu_fix_on_off_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_imu_fix_on_off_ -> setWordWrap(true);
	label_imu_fix_on_off_ -> setFont(QFont("FreeSans", 72));
	label_imu_fix_on_off_ -> setVisible(true);
	gridLayout -> addWidget(label_imu_fix_on_off_, 20, 0, 1, 1);
	nh_.getParam("/imu_not_connected_file", imu_not_connected_file_);
	qimu_not_connected_file_=imu_not_connected_file_.data();
	movie_imu_fix_off_ = new QMovie(qimu_not_connected_file_);
	label_imu_fix_on_off_ -> setMovie(movie_imu_fix_off_);
	movie_imu_fix_off_ -> start();
	nh_.getParam("/imu_connected_file", imu_connected_file_);
	qimu_connected_file_=imu_connected_file_.data();
	movie_imu_fix_on_ = new QMovie(qimu_connected_file_);

	/* BBR status label */

	label_bbr_status_ = new QLabel;
	label_bbr_status_ -> setAlignment(Qt::AlignLeft);
	label_bbr_status_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_bbr_status_ -> setWordWrap(true);
	label_bbr_status_ -> setFont(QFont("FreeSans", 72));
	label_bbr_status_ -> setVisible(true);
	gridLayout -> addWidget(label_bbr_status_, 20, 1, 1, 1);
	nh_.getParam("/bbr_unknown_file", bbr_unknown_file_);
	qbbr_unknown_file_=bbr_unknown_file_.data();
	bbr_unknown_.load(qbbr_unknown_file_);
	label_bbr_status_ -> setPixmap(bbr_unknown_);
	label_bbr_status_ -> show();

	nh_.getParam("/bbr_full_file", bbr_full_file_);
	qbbr_full_file_=bbr_full_file_.data();
	movie_bbr_full_ = new QMovie(qbbr_full_file_);
	nh_.getParam("/bbr_no_drive_in_bay_file", bbr_no_drive_in_bay_file_);
	qbbr_no_drive_in_bay_file_=bbr_no_drive_in_bay_file_.data();
	movie_bbr_no_drive_in_bay_ = new QMovie(qbbr_no_drive_in_bay_file_);
	nh_.getParam("/bbr_ok_0_file", bbr_ok_0_file_);
	qbbr_ok_0_file_=bbr_ok_0_file_.data();
	bbr_ok_0_.load(qbbr_ok_0_file_);
	nh_.getParam("/bbr_ok_1_file", bbr_ok_1_file_);
	qbbr_ok_1_file_=bbr_ok_1_file_.data();
	bbr_ok_1_.load(qbbr_ok_1_file_);
	nh_.getParam("/bbr_ok_2_file", bbr_ok_2_file_);
	qbbr_ok_2_file_=bbr_ok_2_file_.data();
	bbr_ok_2_.load(qbbr_ok_2_file_);
	nh_.getParam("/bbr_ok_3_file", bbr_ok_3_file_);
	qbbr_ok_3_file_=bbr_ok_3_file_.data();
	bbr_ok_3_.load(qbbr_ok_3_file_);
	nh_.getParam("/bbr_ok_4_file", bbr_ok_4_file_);
	qbbr_ok_4_file_=bbr_ok_4_file_.data();
	bbr_ok_4_.load(qbbr_ok_4_file_);
	nh_.getParam("/bbr_stop_file_", bbr_stop_file_);
	qbbr_stop_file_=bbr_stop_file_.data();
	bbr_stop_.load(qbbr_stop_file_);

	/* Time machine status label */

	label_time_machine_ = new QLabel;
	label_time_machine_ -> setAlignment(Qt::AlignLeft);
	label_time_machine_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_time_machine_ -> setWordWrap(true);
	label_time_machine_ -> setFont(QFont("FreeSans", 72));
	label_time_machine_ -> setVisible(true);
	gridLayout -> addWidget(label_time_machine_, 20, 2, 1, 1);
	nh_.getParam("/time_machine_error_file", time_machine_error_file_);
	qtime_machine_error_file_=time_machine_error_file_.data();
	time_machine_error_.load(qtime_machine_error_file_);
	label_time_machine_ -> setPixmap(time_machine_error_);
	label_time_machine_ -> show();
	nh_.getParam("/time_machine_ready_file", time_machine_ready_file_);
	qtime_machine_ready_file_=time_machine_ready_file_.data();
	movie_time_machine_ready_ = new QMovie(qtime_machine_ready_file_);
	nh_.getParam("/time_machine_writing_file", time_machine_writing_file_);
	qtime_machine_writing_file_=time_machine_writing_file_.data();
	movie_time_machine_writing_ = new QMovie(qtime_machine_writing_file_);

	/* Lidar status label */

	label_lidar_status_ = new QLabel;
	label_lidar_status_ -> setAlignment(Qt::AlignLeft);
	label_lidar_status_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_lidar_status_ -> setWordWrap(true);
	label_lidar_status_ -> setFont(QFont("FreeSans", 72));
	label_lidar_status_ -> setVisible(true);
	gridLayout -> addWidget(label_lidar_status_, 20, 3, 1, 1);
	nh_.getParam("/lidar_not_connected_file", lidar_not_connected_file_);
	qlidar_not_connected_file_=lidar_not_connected_file_.data();
	lidar_status_off_.load(qlidar_not_connected_file_);
	label_lidar_status_ -> setPixmap(lidar_status_off_);
	label_lidar_status_ -> show();
	nh_.getParam("/lidar_connected_file", lidar_connected_file_);
	qlidar_connected_file_=lidar_connected_file_.data();
	lidar_status_on_.load(qlidar_connected_file_);

	/* Navigation status label */

	navigation_image_label_ = new QLabel;
	navigation_image_label_ -> setAlignment(Qt::AlignLeft);
	navigation_image_label_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	navigation_image_label_ -> setWordWrap(true);
	navigation_image_label_ -> setFont(QFont("FreeSans", 72));
	navigation_image_label_ -> setVisible(true);
	gridLayout -> addWidget(navigation_image_label_, 22, 0, 1, 1);
	nh_.getParam("/navigation_unknown_file",navigation_unknown_file_);
	qnavigation_unknown_file_=navigation_unknown_file_.data();
	navigation_unknown_.load(qnavigation_unknown_file_);
	navigation_image_label_ -> setPixmap(navigation_unknown_);
	navigation_image_label_ -> show();
	nh_.getParam("/navigation_paused_file", navigation_paused_file_);
	qnavigation_paused_file_=navigation_paused_file_.data();
	navigation_paused_.load(qnavigation_paused_file_);
	nh_.getParam("/navigation_stopped_file", navigation_stopped_file_);
	qnavigation_stopped_file_=navigation_stopped_file_.data();
	navigation_stopped_.load(qnavigation_stopped_file_);
	nh_.getParam("/navigation_running_file", navigation_running_file_);
	qnavigation_running_file_=navigation_running_file_.data();
	navigation_running_.load(qnavigation_running_file_);

	/* Covariance status label */

	covariance_image_label_ = new QLabel;
	covariance_image_label_ -> setAlignment(Qt::AlignLeft);
	covariance_image_label_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	covariance_image_label_ -> setWordWrap(true);
	covariance_image_label_ -> setFont(QFont("FreeSans", 72));
	covariance_image_label_ -> setVisible(true);
	gridLayout -> addWidget(covariance_image_label_, 22, 1, 1, 1);
	nh_.getParam("/covariance_unknown_file", covariance_unknown_file_);
	qcovariance_unknown_file_=covariance_unknown_file_.data();
	covariance_unknown_.load(qcovariance_unknown_file_);
	covariance_image_label_ -> setPixmap(covariance_unknown_);
	covariance_image_label_ -> show();
	nh_.getParam("/covariance_ok_file", covariance_ok_file_);
	qcovariance_ok_file_=covariance_ok_file_.data();
	covariance_good_.load(qcovariance_ok_file_);
	nh_.getParam("/covariance_error_file", covariance_error_file_);
	qcovariance_error_file_=covariance_error_file_.data();
	covariance_error_.load(qcovariance_error_file_);
	nh_.getParam("/covariance_poor_file", covariance_poor_file_);
	qcovariance_poor_file_=covariance_poor_file_.data();
	covariance_poor_.load(qcovariance_poor_file_);

	/* System monitor status label */

	label_system_monitor_ = new QLabel;
	label_system_monitor_ -> setAlignment(Qt::AlignLeft);
	label_system_monitor_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_system_monitor_ -> setWordWrap(true);
	label_system_monitor_ -> setFont(QFont("FreeSans", 72));
	label_system_monitor_ -> setVisible(true);
	gridLayout -> addWidget(label_system_monitor_, 22, 2, 1, 1);
	nh_.getParam("/ads_temp_unknown_file", ads_temp_unknown_file_);
	qads_temp_unknown_file_=ads_temp_unknown_file_.data();
	ads_temp_unknown_.load(qads_temp_unknown_file_);
	label_system_monitor_ -> setPixmap(ads_temp_unknown_);
	label_system_monitor_ -> show();
	nh_.getParam("/ads_temp_ok", ads_temp_ok_file_);
	qads_temp_ok_file_=ads_temp_ok_file_.data();
	ads_temp_ok_.load(qads_temp_ok_file_);
	nh_.getParam("/cpu_temp_high", cpu_temp_high_file_);
	qcpu_temp_high_file_=cpu_temp_high_file_.data();
	cpu_temp_high_.load(qcpu_temp_high_file_);
	nh_.getParam("/gpu_temp_high_file", gpu_temp_high_file_);
	qgpu_temp_high_file_=gpu_temp_high_file_.data();
	gpu_temp_high_.load(qgpu_temp_high_file_);
	nh_.getParam("ads_temp_high_file", ads_temp_high_file_);
	qads_temp_high_file_=ads_temp_high_file_.data();
	ads_temp_high_.load(qads_temp_high_file_);

	/* Loading arms status label */

	label_loading_arms_ = new QLabel;
	label_loading_arms_ -> setAlignment(Qt::AlignLeft);
	label_loading_arms_ -> setStyleSheet("font-weight: bold;color: black; font-size:14pt");
	label_loading_arms_ -> setWordWrap(true);
	label_loading_arms_ -> setFont(QFont("FreeSans", 72));
	label_loading_arms_ -> setVisible(true);
	gridLayout -> addWidget(label_loading_arms_, 22, 3, 1, 1);
	nh_.getParam("/loading_arms_unknown_file", loading_arms_unknown_file_);
	qloading_arms_unknown_file_=loading_arms_unknown_file_.data();
	loading_arms_unknown_.load(qloading_arms_unknown_file_);
	label_loading_arms_ -> setPixmap(loading_arms_unknown_);
	label_loading_arms_ -> show();
	nh_.getParam("/loading_arms_deployed_file", loading_arms_deployed_file_);
	qloading_arms_deployed_file_=loading_arms_deployed_file_.data();
	loading_arms_deployed_.load(qloading_arms_deployed_file_);
	nh_.getParam("/loading_arms_retracted_file", loading_arms_retracted_file_);
	qloading_arms_retracted_file_=loading_arms_retracted_file_.data();
	loading_arms_retracted_.load(qloading_arms_retracted_file_);

	/* Message QEditline */

	debug_message_edit_ = new QTextEdit;
	gridLayout -> addWidget(debug_message_edit_, 23, 0, 1, 4);
	debug_message_edit_ -> setFixedSize(600, 50);

	setLayout(gridLayout);


	/*is_rviz_locked_unlocked_.data = true;
	lock_unlock_rviz_status_publisher_.publish(is_rviz_locked_unlocked_);*/

	/*General waypoints path planner function calls*/

	publish_waypoint_paths();
	wppngzm.publish_empty_no_go_zone();
	//wppngzm.load_waypoints_and_nogozones(filename_, left_lane_filename_, right_lane_filename_);
	//wppngzm.load_route_process(filename_, left_lane_filename_, right_lane_filename_);
	get_routes_from_file();

	is_rviz_locked_unlocked_.data = true;
		lock_unlock_rviz_status_publisher_.publish(is_rviz_locked_unlocked_);

	/* Entry dialog for supervisor */

	bool ok;
	QString text = QInputDialog::getText(0, "Safety Supervisor ID",
			"Safety Supervisor Name:", QLineEdit::Normal,
			"", & ok);
	if (ok && !text.isEmpty()) {

		QMessageBox::information(0, "Safety Supervisor Name ",
				QString("Supervisor %1 is logged in").arg(QString(text)));
		safety_supervisor_msg.data = text.toStdString().c_str();

		safety_supervisor_name_publisher_.publish(safety_supervisor_msg);
	} else if (!ok) {
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(this, "DO YOU WANT TO QUIT", "'CANCEL' QUITS THE NAVIGATION PROCESS,'OK' STARTS NAVIGATION AFTER LOGIN",
				QMessageBox::Cancel | QMessageBox::Ok);
		if (reply == QMessageBox::Cancel) {
			nh_.getParam("/quit_process_file", quit_process_file_);
			command = quit_process_file_;
			system(command.c_str());

		} else if (reply == QMessageBox::Ok) {
			QString text = QInputDialog::getText(0, "Safety Supervisor ID",
					"Safety Supervisor Name:", QLineEdit::Normal,
					"", & ok);
			if (ok && !text.isEmpty()) {

				QMessageBox::information(0, "Safety Supervisor Name ",
						QString("Supervisor %1 is logged in").arg(QString(text)));
				safety_supervisor_msg.data = text.toStdString().c_str();

				safety_supervisor_name_publisher_.publish(safety_supervisor_msg);
			} else {
				nh_.getParam("/quit_process_file", quit_process_file_);
				command = quit_process_file_;
				system(command.c_str());
			}
		}
	} else if (text.isEmpty()) {
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(this, "SUPERVISOR ID IS MANDATORY", "'CANCEL' QUITS THE NAVIGATION PROCESS,'OK' STARTS NAVIGATION AFTER LOGIN",
				QMessageBox::Cancel | QMessageBox::Ok);
		if (reply == QMessageBox::Cancel) {
			nh_.getParam("/quit_process_file", quit_process_file_);
			command = quit_process_file_;
			system(command.c_str());
		} else if (reply == QMessageBox::Ok) {
			QString text = QInputDialog::getText(0, "Safety Supervisor ID",
					"Safety Supervisor Name:", QLineEdit::Normal,
					"", & ok);
			if (ok && !text.isEmpty()) {

				QMessageBox::information(0, "Safety Supervisor Name ",
						QString("Supervisor %1 is logged in").arg(QString(text)));

				safety_supervisor_msg.data = text.toStdString().c_str();

				safety_supervisor_name_publisher_.publish(safety_supervisor_msg);
			} else {
				nh_.getParam("/quit_process_file", quit_process_file_);
				command = quit_process_file_;
				system(command.c_str());
			}
		}
	}

	/* Label for safety supervisor name*/

	QLabel * displaySupervior = new QLabel;
	displaySupervior -> setAlignment(Qt::AlignRight);
	displaySupervior -> setFont(QFont("FreeSans", 72));
	displaySupervior -> setStyleSheet("font-weight: bold;color: black; font-size:12pt");
	displaySupervior -> setText("SUPERVISED BY ");
	gridLayout -> addWidget(displaySupervior, 0, 2, 1, 1);

	QLabel * displaySuperviorName = new QLabel;
	displaySuperviorName -> setAlignment(Qt::AlignLeft);
	displaySuperviorName -> setFont(QFont("FreeSans", 72));
	displaySuperviorName -> setStyleSheet("font-weight: bold;color: black; font-size:12pt");
	displaySuperviorName -> setText(text.toUpper());
	gridLayout -> addWidget(displaySuperviorName, 0, 3, 1, 1);

	waypoints_size_ = wppngzm.main_lane_marker_names.size();

	set_button_state(start_navigation_,true);
	set_button_state(cancel_navigation_,true);
	set_button_state(pause_release_,true);
	set_button_state(vehicle_pose_estimate_,true);
	set_button_state(change_map_,false);
	set_button_state(load_waypoints_file_,false);
	set_button_state(compute_segment_,false);
	set_button_state(clear_waypoint_markers_,false);
	set_button_state(create_waypoint_file_,false);
	set_button_state(place_lidar_waypoints_,false);
	set_button_state(place_gps_waypoints_,false);
	set_button_state(left_lane_switch_,true);
	set_button_state(right_lane_switch_,true);
	set_button_state(create_route_file_,false);
	set_button_state(load_route_file_,false);
	set_button_state(set_route_file_,false);
	set_button_state(run_route_1_file_,true);
	set_button_state(run_route_2_file_,true);
	set_button_state(run_route_3_file_,true);
	set_button_state(run_route_4_file_,true);
	set_button_state(delete_no_go_zone_markers_in_file_and_rviz_,false);
		}
/*WaypointsPathPlanner::~WaypointsPathPlanner()
{

}*/

void WaypointsPathPlanner::publish_waypoint_paths()
{
	waypoints_file_path_.mainlanefilepath = filename_;
	waypoints_file_path_.leftlanefilepath = left_lane_filename_;
	waypoints_file_path_.rightlanefilepath = right_lane_filename_;
	file_path_publisher_.publish(waypoints_file_path_);

	ROS_INFO_STREAM("Published Waypoint File Paths:");
	ROS_INFO_STREAM("  Main Lane: " << filename_);
	ROS_INFO_STREAM("  Left Lane: " << left_lane_filename_);
	ROS_INFO_STREAM("  Right Lane: " << right_lane_filename_);
}

void WaypointsPathPlanner::set_button_state(QPushButton* button, bool enabled_or_not)
{
	button->setEnabled(enabled_or_not);
	button->setFixedSize(150, 28);
	if (enabled_or_not == 1)
	{
		button->setStyleSheet("font-weight: bold;color: black; background-color: green; font-size:9pt");
	} else
	{
		button->setStyleSheet("font-weight: bold;color: black; background-color: grey; font-size:9pt");
	}
	button->setFont(QFont("FreeSans"));

}
void WaypointsPathPlanner::save(rviz::Config config) const {
	ROS_INFO_STREAM("Saving configuration");
	rviz::Panel::save(config);
	config.mapSetValue("TextEntry", QString::fromStdString(std::string("test_field")));
}
void WaypointsPathPlanner::load(const rviz::Config & config) {
	rviz::Panel::load(config);
	QString text_entry;
	ROS_INFO_STREAM("rviz: Initializing the user interaction planning panel");
	if (config.mapGetString("TextEntry", & text_entry)) {
		ROS_INFO_STREAM("Loaded TextEntry with value: " << text_entry.toStdString());
	}
	ROS_INFO_STREAM("rviz Initialization Finished reading config file");
}
void WaypointsPathPlanner::load_route()
{
	if(does_main_lane_working_folder_exist_ && does_right_lane_working_folder_exist_ && does_left_lane_working_folder_exist_)
	{
		wppngzm.load_route_process(filename_, left_lane_filename_, right_lane_filename_);
	}
}

/* Q-Slot's Definitions*/
void WaypointsPathPlanner::set_poseestimate_qslot() {
	bool is_pose_estimate_set = wpplm.publish_pose_estimate_from_waypoints_file(filename_);
	if (is_pose_estimate_set)
	{
		debug_message_edit_ -> clear();
		debug_message_edit_ -> setText("Pose estimate done successfully ");
		QMessageBox::information(this, "INFORMATION", "Pose estimate done successfully", QMessageBox::Ok);
	}
	else
	{
		debug_message_edit_ -> clear();
		debug_message_edit_ -> setText("Pose estimate cannot be done! No points in file");
		QMessageBox::information(this, "WARNING", "No points in file to do a pose estimate", QMessageBox::Ok);
	}
}
void WaypointsPathPlanner::load_newmap_qslot(){
	nh_.getParam("/map_file_path", map_file_path_);
	qmap_file_path_=map_file_path_.data();
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this, "WARNING", "LOADING A NEW MAP WILL CLEAR WAYPOINTS AND NO GO ZONES IN RVIZ! DO YOU WANT TO PROCEED?", QMessageBox::Yes | QMessageBox::No);
	if (reply == QMessageBox::Yes) {
		QString file_path=QFileDialog::getOpenFileName(this, "Map", qmap_file_path_, "PCD Files (*.pcd)");
		map_file_path_msg.data = file_path.toStdString().c_str();
		map_file_path_publisher_.publish(map_file_path_msg);
	}
}
void WaypointsPathPlanner::start_av_navigation_qslot(void)
{
	if (is_av_navigating_msg_.data == false) 
	{
		if (boot_safe || true)
		{
			std::cout << "Does waypoint exist" << wppngzm.does_waypoints_exist_in_main_lane_ << std::endl;
			if (wppngzm.does_waypoints_exist_in_main_lane_ == true) 
			{
				ROS_INFO_STREAM("start Navigation from panel");
				std::vector < waypoints_path_planning_tool::SectionTrajectory > route;
				waypoints_path_planning_file_management::getlistofwaypoints list;
				std::cout<<"mainlanefilename"<<filename_<<std::endl;
				list= wppfm.get_list_of_waypoints_from_file(filename_,route);
				route=list.Section;
				std::cout<<"route=list.Section;"<<route.size()<<std::endl;
				std::cout<<"list.doeswaypointsexist"<<list.doeswaypointsexist<<std::endl;
				if (list.doeswaypointsexist)
				{
					navigation_started_=wpplm.start_av_navigation_process(route);
					if(navigation_started_==true)
					{
						ROS_INFO("MESSAGE WAS SENT PROPERLY TO START NAVIGATION");
						debug_message_edit_ -> clear();
						debug_message_edit_ -> setText("Navigation Started");

						set_button_state(cancel_navigation_, true);
						set_button_state(start_navigation_, false);
						//click pause release to ensure that is was not left hanging from last time
						pause_release_->click();
					}
					else
					{
						QMessageBox::information(this, "CAUTION", "/start_navigation service has FAILED", QMessageBox::Ok);
						debug_message_edit_ -> clear();
						debug_message_edit_ -> setText("Navigation has some problems");
					}
				}
			} 
			else 
			{
				QMessageBox::information(this, "CAUTION", "AT LEAST 2 WAYPOINTS REQUIRED FOR NAVIGATION", QMessageBox::Ok);
			}

		}
		else
		{
			QMessageBox::information(this, "CAUTION", "BLACKBOX RECORDER NOT ACTIVE", QMessageBox::Ok);
		}
	} 
	// else 
	// {
	//     QMessageBox::information(this, "CAUTION", "POD ALREADY IN A ROUTE! WAIT FOR THE ROUTE TO COMPLETE .. OR NAVIGATION HAS SOME PROBLEMS", QMessageBox::Ok);
	// }
 }
void WaypointsPathPlanner::cancel_navigation_qslot(void)
{
	bool navigation_cancelled = wpplm.cancel_av_navigation_process(navigation_started_);
	std::cout<<"CANCELLING NAVIGATION"<<navigation_cancelled<<std::endl;
	if (navigation_cancelled == false) {
		debug_message_edit_ -> clear();
		debug_message_edit_ -> setText("Navigation is not currently running");
	} else {
		debug_message_edit_ -> clear();
		debug_message_edit_ -> setText("Navigation Cancelled");
		ROS_INFO("cancelNavigation_Panel executed");
		set_button_state(cancel_navigation_, false);
		set_button_state(start_navigation_, true);
		start_navigation_ -> setEnabled(true); // Start Button is enabled once the navigation is cancelled
	}
}
void WaypointsPathPlanner::lock_unlock_rvizedit_qslot()
{
	if (is_av_navigating_msg_.data == false) {
		if (toggle_lock_unlock_ -> isChecked()) {
			toggle_lock_unlock_ -> setIcon(QIcon(qtoggle_button_unlock_file_));
			set_button_state(start_navigation_,false);
			set_button_state(cancel_navigation_,false);
			set_button_state(pause_release_,false);
			set_button_state(vehicle_pose_estimate_,false);
			set_button_state(change_map_,true);
			set_button_state(load_waypoints_file_,true);
			set_button_state(compute_segment_,true);
			set_button_state(clear_waypoint_markers_,true);
			set_button_state(create_waypoint_file_,true);
			set_button_state(place_lidar_waypoints_,true);
			set_button_state(place_gps_waypoints_,true);
			set_button_state(left_lane_switch_,false);
			set_button_state(right_lane_switch_,false);
			set_button_state(create_route_file_,true);
			set_button_state(load_route_file_,true);
			set_button_state(set_route_file_,true);
			set_button_state(run_route_1_file_,false);
			set_button_state(run_route_2_file_,false);
			set_button_state(run_route_3_file_,false);
			set_button_state(run_route_4_file_,false);
			set_button_state(delete_no_go_zone_markers_in_file_and_rviz_,true);
			is_rviz_locked_unlocked_.data = false;
			lock_unlock_rviz_status_publisher_.publish(is_rviz_locked_unlocked_);
		}
		else
		{
			is_rviz_locked_unlocked_.data = true;
			lock_unlock_rviz_status_publisher_.publish(is_rviz_locked_unlocked_);
			toggle_lock_unlock_ -> setIcon(QIcon(qtoggle_button_lock_file_));
			set_button_state(start_navigation_,true);
			set_button_state(cancel_navigation_,true);
			set_button_state(pause_release_,true);
			set_button_state(vehicle_pose_estimate_,true);
			set_button_state(change_map_,false);
			set_button_state(load_waypoints_file_,false);
			set_button_state(compute_segment_,false);
			set_button_state(clear_waypoint_markers_,false);
			set_button_state(create_waypoint_file_,false);
			set_button_state(place_lidar_waypoints_,false);
			set_button_state(place_gps_waypoints_,false);
			set_button_state(left_lane_switch_,true);
			set_button_state(right_lane_switch_,true);
			set_button_state(create_route_file_,false);
			set_button_state(load_route_file_,false);
			set_button_state(set_route_file_,false);
			set_button_state(run_route_1_file_,true);
			set_button_state(run_route_2_file_,true);
			set_button_state(run_route_3_file_,true);
			set_button_state(run_route_4_file_,true);
			set_button_state(delete_no_go_zone_markers_in_file_and_rviz_,false);
			/*is_rviz_locked_unlocked_.data = true;
		lock_unlock_rviz_status_publisher_.publish(is_rviz_locked_unlocked_);*/
			//wppngzm.compute_segment_process(wppfm.filename_,wppfm.left_lane_filename_,wppfm.right_lane_filename_);
		}
	}
	else {
		QMessageBox::information(this, "CAUTION", "AV NAVIGATING", QMessageBox::Ok);
	}

}
void WaypointsPathPlanner::switch_to_leftlane_qslot()
{
	set_button_state(left_lane_switch_,false);
	set_button_state(right_lane_switch_,false);
	wppngzm.lane_switching_process("left_lane", file_location_of_vehicle_global_plan, navigation_started_, line_number_of_vehicle_pose_in_the_global_plan_, current_vehicle_pose_from_local_planner_);

}
void WaypointsPathPlanner::switch_to_rightlane_qslot()
{
	set_button_state(left_lane_switch_,false);
	set_button_state(right_lane_switch_,false);
	ROS_INFO("LOOK IN WPPNGZM");
	wppngzm.lane_switching_process("right_lane", file_location_of_vehicle_global_plan, navigation_started_, line_number_of_vehicle_pose_in_the_global_plan_, current_vehicle_pose_from_local_planner_);

}
void WaypointsPathPlanner::place_lidar_waypoints_qslot()
{
	if (is_rviz_locked_unlocked_.data == false) {
		if (is_av_navigating_msg_.data == false) {
			if(check_localization_pose_==true)
			{
				new_pose_ = vehicle_pose_;
				if ((vehicle_pose_.pose.position.x == old_pose_.pose.position.x) && (waypoints_size_ != 0)) {
					QMessageBox::information(this, "CAUTION", "CANNOT PLACE TWO WAYPOINTS IN THE SAME POSITION ", QMessageBox::Ok);
				} else {
					if (wppngzm.waypoint_manipulation_panel_ != NULL) {
						wppngzm.save_button_ -> click();
						wppngzm.create_waypoint_markers(vehicle_pose_, "main_lane", "main_lane_waypoint", -1,-1,0);
						wppngzm.create_waypoint_markers(vehicle_pose_,"left_lane","left_lane_waypoint", -1,-1,0);
						wppngzm.create_waypoint_markers(vehicle_pose_,"right_lane","right_lane_waypoint", -1,-1,0);
						wppngzm.save_waypoints_and_nogozones();
						wppngzm.clear_no_go_zone_objects_before_loading_file();
						wppngzm.clear_waypoints_descriptors_arrays();
						wppngzm.load_waypoints_and_nogozones(filename_,left_lane_filename_,right_lane_filename_);
					} else {
						wppngzm.create_waypoint_markers(vehicle_pose_, "main_lane", "main_lane_waypoint", -1,-1,0);
						wppngzm.create_waypoint_markers(vehicle_pose_,"left_lane","left_lane_waypoint", -1,-1,0);
						wppngzm.create_waypoint_markers(vehicle_pose_,"right_lane","right_lane_waypoint", -1,-1,0);
						wppngzm.save_waypoints_and_nogozones();
						wppngzm.clear_no_go_zone_objects_before_loading_file();
						wppngzm.clear_waypoints_descriptors_arrays();
						wppngzm.load_waypoints_and_nogozones(filename_,left_lane_filename_,right_lane_filename_);
					}
					debug_message_edit_ -> clear();
					debug_message_edit_ -> setText("The waypoint is successfully placed in RVIZ and saved in the segment file");
					if (wppngzm.waypoint_manipulation_panel_ != NULL) {
						if (waypoints_size_ > 0) {
							wppngzm.index_spinbox_ -> setRange(0, waypoints_size_ - 1);
							wppngzm.from_waypoint_spinbox_ -> setRange(0, waypoints_size_ - 1);
							wppngzm.to_waypoint_spinbox_ -> setRange(0, waypoints_size_ - 1);
						} else if (waypoints_size_ == 1) {
							wppngzm.index_spinbox_ -> setRange(0, waypoints_size_);
							wppngzm.from_waypoint_spinbox_ -> setRange(0, waypoints_size_ - 1);
							wppngzm.to_waypoint_spinbox_ -> setRange(0, waypoints_size_ - 1);
						}
					} else {
					}
				}
				old_pose_ = new_pose_;
			} else {
				debug_message_edit_ -> clear();
				debug_message_edit_ -> setText("No waypoints placed as there is no vehicle pose from amcl node");

				check_localization_pose_ = false;
				QMessageBox::information(this, "CAUTION", "THE POSE MESSAGE IS NOT SUBSCRIBED. CANNOT PLACE WAYPOINT", QMessageBox::Ok);

			}
		} else {
			QMessageBox::information(this, "CAUTION", "AV NAVIGATING !!! WAIT FOR THE ROUTE TO COMPLETE, CANNOT CLEAR WAYPOINTS WHILE AV IS NAVIGATING!", QMessageBox::Ok);
		}
	} else {
		QMessageBox::information(this, "INFORMATION", "UNLOCK RVIZ USING THE LOCK/UNLOCK BUTTON TO PLOT NEW WAYPOINTS", QMessageBox::Ok);
	}
}

void WaypointsPathPlanner::place_gps_waypoints_qslot()
{
	if (check_gps_pod_pose_ == true) {
		if (wppngzm.waypoint_manipulation_panel_ != NULL) {
			wppngzm.save_button_ -> click();
			wppngzm.create_waypoint_markers(vehicle_pose_, "main_lane", "main_lane_waypoint", -1,-1,0);
			wppngzm.create_waypoint_markers(vehicle_pose_,"left_lane","left_lane_waypoint", -1,-1,0);
			wppngzm.create_waypoint_markers(vehicle_pose_,"right_lane","right_lane_waypoint", -1,-1,0);
			wppngzm.save_waypoints_and_nogozones();
			wppngzm.clear_no_go_zone_objects_before_loading_file();
			wppngzm.clear_waypoints_descriptors_arrays();
			wppngzm.load_waypoints_and_nogozones(filename_,left_lane_filename_,right_lane_filename_);

		} else {
			wppngzm.create_waypoint_markers(vehicle_pose_, "main_lane", "main_lane_waypoint", -1,-1,0);
			wppngzm.create_waypoint_markers(vehicle_pose_,"left_lane","left_lane_waypoint", -1,-1,0);
			wppngzm.create_waypoint_markers(vehicle_pose_,"right_lane","right_lane_waypoint", -1,-1,0);
			wppngzm.save_waypoints_and_nogozones();
			wppngzm.clear_no_go_zone_objects_before_loading_file();
			wppngzm.clear_waypoints_descriptors_arrays();
			wppngzm.load_waypoints_and_nogozones(filename_,left_lane_filename_,right_lane_filename_);
		}
		debug_message_edit_ -> clear();
		debug_message_edit_ -> setText("The waypoint is successfully placed RVIZ and saved in the segment file");
		if (wppngzm.waypoint_manipulation_panel_ != NULL) {
			if (waypoints_size_ > 0) {
				wppngzm.index_spinbox_ -> setRange(0, waypoints_size_ - 1);
				wppngzm.from_waypoint_spinbox_ -> setRange(0, waypoints_size_ - 1);
				wppngzm.to_waypoint_spinbox_ -> setRange(0, waypoints_size_ - 1);
			} else if (waypoints_size_ == 1) {
				wppngzm.index_spinbox_ -> setRange(0, waypoints_size_);
				wppngzm.from_waypoint_spinbox_ -> setRange(0, waypoints_size_ - 1);
				wppngzm.to_waypoint_spinbox_ -> setRange(0, waypoints_size_ - 1);
			}
		} else {
		}
	}
	else {
		std::cout << "No GPS Pose" << std::endl;
		check_gps_pod_pose_ = false;
		debug_message_edit_ -> clear();
		debug_message_edit_ -> setText("No waypoints placed as there is no pose from /odometry/filetered topic");

		QMessageBox::information(this, "CAUTION", "THE POSE MESSAGE IS NOT SUBSCRIBED. CANNOT PLACE WAYPOINT.", QMessageBox::Ok);
	}
}

/* Route file to navigate multiple segments*/
void WaypointsPathPlanner::create_route_file_qslot(){
	if (if_route_file_exist_) {
		QDir::setCurrent("/tmp");
		boost::filesystem::path my_path(routes_filename_);
		std::cout << "Boost Working" << my_path << std::endl;
		size_t found = routes_filename_.find_last_of("/\\");
		std::string path = routes_filename_.substr(0, found);
		QDir::setCurrent(path.data());
	}
	QString filename = QFileDialog::getSaveFileName(this,
			tr("Save Route"), "/home/aurrigo/code/ros/Launchers/Pod/Routes/untitled.yaml",
			tr("Routes (*.yaml);;All Files (*)"));
	if (filename.isEmpty()) {
		// Do nothing
	} else {
		QFile file(filename);
		routes_filename_ = filename.toStdString().c_str();
		//routes_file_array_.clear();
		route_info_array_.routeinfodata.clear();
		save_route();
		if (!file.open(QIODevice::ReadWrite | QIODevice::Text)) {
			QMessageBox::information(this, tr("Unable to open file"),
					file.errorString());
		}
	}
}

void WaypointsPathPlanner::create_waypoint_file_qslot()
{
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this, "Create New Segment file", "DO YOU WANT TO CREATE A NEW SEGMENT FILE? ",
			QMessageBox::Yes | QMessageBox::No);
	if (reply == QMessageBox::Yes) {// this section needs doing
		if (wppngzm.waypoint_manipulation_panel_ != NULL) {
			wppngzm.waypoint_manipulation_panel_ -> close();
			wppngzm.waypoint_manipulation_panel_ = NULL;
			//timer_for_buffer_array_.stop();
		}
		QMessageBox::information(this, "WARNING", "SAVE YOUR SEGMENT FILE WITHIN A NEW FOLDER. DO NOT SAVE MORE THAN ONE SEGMENT FILE IN THE SAME FOLDER.", QMessageBox::Ok);
		wppngzm.clear_waypoints_descriptors_arrays();
		wppngzm.clear_no_go_zone_objects_before_loading_file();
		if (ifWorkingFileExist_) {
			QDir::setCurrent("/tmp");
			boost::filesystem::path my_path(filename_);
			std::cout << "Boost Working" << my_path << std::endl;
			size_t found = filename_.find_last_of("/\\");
			std::string path = filename_.substr(0, found);
			QDir::setCurrent(path.data());
		}
		QString filename = QFileDialog::getSaveFileName(this,
				tr("Save As"), "/home/aurrigo-ads/code/ros/Launchers/Pod/3D_ACS_LAUNCHERS/LAUNCHERS/Waypoints/Default/default.yaml",
				tr("WAYPOINTS (*.yaml)"));
		if (filename.isEmpty()) {
			wppngzm.load_waypoints_and_nogozones(filename_, left_lane_filename_, right_lane_filename_);
			debug_message_edit_ -> clear();
			debug_message_edit_ -> setText("Failed to create new segment file. Loaded default segment file");
		} else {
			QFile file(filename);
			filename_=filename.toStdString().c_str();

			ifWorkingFileExist_ = true;
			boost::filesystem::path my_path(filename_);
			std::string directory_left = my_path.parent_path().string() + "/LeftLane";
			std::string directory_right = my_path.parent_path().string() + "/RightLane";
			left_lane_filename_ = directory_left + "/leftlane.yaml";
			right_lane_filename_ = directory_right + "/rightlane.yaml";
			publish_waypoint_paths();
			std::fstream output_fstream_left, output_fstream_right;
			if (boost::filesystem::create_directory(directory_left)) {
				output_fstream_left.open(left_lane_filename_, std::ios_base::out);
				if (!output_fstream_left.is_open()) {
					std::cerr << "Failed to open " << left_lane_filename_ << '\n';
				} else {
					std::cout << "LeftLane file created" << std::endl;
				}
			} else {
				QMessageBox::information(this, "INFORMATION", "LEFT LANE FOLDER CANNOT BE CREATED", QMessageBox::Ok);
			}
			if (boost::filesystem::create_directory(directory_right)) {
				output_fstream_right.open(right_lane_filename_, std::ios_base::out);
				if (!output_fstream_right.is_open()) {
					std::cerr << "Failed to open " << right_lane_filename_ << '\n';
				} else {
					std::cout << "RightLane file created" << std::endl;
				}
			} else {
				QMessageBox::information(this, "INFORMATION", "RIGHT LANE FOLDER CANNOT BE CREATED", QMessageBox::Ok);

			}
			//wppngzm.load_waypoints_and_nogozones(filename_, left_lane_filename_, right_lane_filename_);
			//wppngzm.save_waypoints_and_nogozones();
			debug_message_edit_ -> clear();
			debug_message_edit_ -> setText("New segment file created successfully");
			if (!file.open(QIODevice::ReadWrite | QIODevice::Text)) {
				QMessageBox::information(this, tr("Unable to open file"),
						file.errorString());
			}
		}
	}
	else {
		QMessageBox::information(this, "INFORMATION", "NO CHANGES HAS BEEN DONE", QMessageBox::Ok);
	}
}

void WaypointsPathPlanner::load_route_file_qslot()
{
	if(routes_manipulation_panel_!=NULL)
	{
		routes_manipulation_panel_==NULL;
		routes_manipulation_panel_->close();
	}
	matrix_map.clear();
	//routes_file_array_.clear();
	route_info_array_.routeinfodata.clear();
	QString route_file = QFileDialog::getOpenFileName(this, "Open a Route file", "/home/aurrigo/code/ros/Launchers/Pod/Routes","Yaml Files (*.yaml)");
	routes_filename_= route_file.toStdString().c_str();
	bool get = get_routes_from_file();
}
std::string WaypointsPathPlanner::int_to_hexstring(int num) {
	std::ostringstream oss;
	oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << num;
	return oss.str();
}
std::string WaypointsPathPlanner::get_substring_after_last_underscore(const std::string& str) {
	auto it = boost::find_last(str, "_");
	if (it.begin() == str.end()) return "no_underscore"; // No underscore found
	return std::string(it.end(), str.end());
}

bool WaypointsPathPlanner::get_routes_from_file() {
	try {
		// Load YAML file directly
		YAML::Node node = YAML::LoadFile(routes_filename_);
		ROS_INFO_STREAM("Accessing Routes From File");

		const YAML::Node& wp_node_tmp = node["Routes"];
		if (!wp_node_tmp) {
			QMessageBox::information(this, "WARNING", "THIS IS NOT A ROUTES FILE", QMessageBox::Ok);
			return false;
		}

		if (wp_node_tmp.size() == 0) {
			QMessageBox::information(this, "WARNING", "NO ROUTES SET IN FILE.", QMessageBox::Ok);
			return false;
		}

		route_info_array_.routeinfodata.resize(wp_node_tmp.size());
		for (size_t i = 0; i < wp_node_tmp.size(); i++) {
			std::string temp, temp1;
			int temp3;

			wp_node_tmp[i]["route"]["Route"] >> temp;
			wp_node_tmp[i]["route"]["Name"] >> temp1;
			wp_node_tmp[i]["route"]["Index"] >> temp3;

			route_info_array_.routeinfodata[i].index = temp3;
			route_info_array_.routeinfodata[i].route_path = temp;
			route_info_array_.routeinfodata[i].route_name = temp1;
		}

		for (const auto& route : route_info_array_.routeinfodata) {
			std::cout << "Routes: " << route.route_path << std::endl;
		}

		std::cout << "Routes successfully loaded." << std::endl;

		matrix_map = read_matrix_from_yaml(routes_filename_);

		return true;
	} catch (const YAML::ParserException& e) {
		std::cerr << "YAML ParserException: " << e.what() << std::endl;
		return false;
	} catch (const YAML::RepresentationException& e) {
		std::cerr << "YAML RepresentationException: " << e.what() << std::endl;
		return false;
	} catch (const std::exception& e) {
		std::cerr << "Exception: " << e.what() << std::endl;
		return false;
	}
}

void WaypointsPathPlanner::save_route()
{
	int n = route_info_array_.routeinfodata.size();
	std::vector<std::vector<int>> matrix(n, std::vector<int>(n));
	// Fill matrix with some values (e.g., row index * column index)
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			matrix[i][j] = 0;
		}
	}
	std::ofstream ofs(routes_filename_.c_str(), std::ios::out);
	std::cout << route_info_array_.routeinfodata.size() << endl;
	std::string alphabet ="ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	ofs << "Routes:" << std::endl;
	for (unsigned int i = 0; i < (route_info_array_.routeinfodata.size()); i++) {
		std::ostringstream ssa;
		std::string name;
		name += alphabet[i];
		name += alphabet[i + 1];
		ssa << i;
		std::string str = ssa.str();
		ofs << "   " << "- route:" << std::endl;
		ofs << "        Route: " << route_info_array_.routeinfodata.at(i).route_path << std::endl;
		ofs << "        Index: " << i << std::endl;
		ofs << "        Name: " << name <<std::endl;
	}
	/*ofs <<"matrix:\n";
	for (int row = 0; row < table_widget_->rowCount(); ++row) {
		ofs << "  - [";  // YAML array format

		for (int col = 0; col < table_widget_->columnCount(); ++col) {
			QTableWidgetItem *item = table_widget_->item(row, col);
			QString value = (item) ? item->text() : "null";

			ofs << value.toStdString();
			if (col < table_widget_->columnCount() - 1)
				ofs << ", ";  // Comma-separated values
		}

		ofs << "]\n";  // Close YAML list
	}

	ofs.close();*/
	YAML::Node yaml_data;
	for (const auto& entry : matrix_map) {
		yaml_data["RoutesPossibilityMatrix"][entry.first.first][entry.first.second] = entry.second;
	}

	// Write to a YAML file
	std::ofstream fout("route_matrix.yaml");
	ofs << yaml_data;
	ofs.close();
}
void WaypointsPathPlanner::process_route_request(int route_number)
{
	if (wppngzm.is_av_navigating_.data == false) {
		if (route_info_array_.routeinfodata.size() > 0) {
			filename_ = route_info_array_.routeinfodata.at(route_number).route_path;
			boost::filesystem::path filename_path(routes_filename_);
			left_lane_filename_ = filename_path.parent_path().string() + "/LeftLane" + "/leftlane.yaml";
			right_lane_filename_ = filename_path.parent_path().string() + "/RightLane" + "/rightlane.yaml";
			publish_waypoint_paths();
			/*if (counter_for_pose_estimate_ < 1) {
				setPoseEstimateUsingFileWaypoint(route_filename);
				counter_for_pose_estimate_++;
			} else {
				//Do nothing
			}
			 */
			wppngzm.clear_waypoints_descriptors_arrays();
			wppngzm.clear_no_go_zone_objects_before_loading_file();
			wppngzm.load_waypoints_and_nogozones(filename_, left_lane_filename_, right_lane_filename_);
			wppngzm.load_route_process(filename_, left_lane_filename_, right_lane_filename_);
			std::vector < waypoints_path_planning_tool::SectionTrajectory > route;
			waypoints_path_planning_file_management::getlistofwaypoints list;
			list = wppfm.get_list_of_waypoints_from_file(filename_,route);
			route=list.Section;
			std::cout<<"route=list.Section;"<<route.size()<<std::endl;
			std::cout<<"list.doeswaypointsexist"<<list.doeswaypointsexist<<std::endl;
			if (list.doeswaypointsexist) {
				navigation_started_=wpplm.start_av_navigation_process(route);
				if(navigation_started_==true)
				{
					ROS_INFO("MESSAGE WAS SENT PROPERLY TO START NAVIGATION");
					debug_message_edit_ -> clear();
					debug_message_edit_ -> setText("Navigation Started");
				}
				else
				{
					QMessageBox::information(this, "CAUTION", "NAVIGATION HAS SOME PROBLEMS.", QMessageBox::Ok);
					ROS_INFO("START NAVIGATION HAD SOME PROBLEMS!!!. RETURN FALSE");
					debug_message_edit_ -> clear();
					debug_message_edit_ -> setText("Navigation has some problems");
				}
			}
		} else {
			QMessageBox::information(this, "CAUTION", "THERE IS NO ROUTE 0 SET IN THE ROUTES FILE", QMessageBox::Ok);
		}
	} 
	// else {
	// 	QMessageBox::information(this, "CAUTION", "POD ALREADY IN A ROUTE! WAIT FOR THE ROUTE TO COMPLETE .. OR NAVIGATION HAS SOME PROBLEMS", QMessageBox::Ok);
	// }
}
void WaypointsPathPlanner::index_value_changed_qslot(int i)
{
	std::string filename = route_info_array_.routeinfodata.at(i).route_path;
	segment_filename_lineedit_ -> setText(filename.c_str());
}
void WaypointsPathPlanner::add_route_qslot()
{
	waypoints_path_planner::routeinfo temp;
	if(route_info_array_.routeinfodata.size()<4)
	{

		QString route = QFileDialog::getOpenFileName(this, "Open a Segment file to be set as route", "/home/aurrigo/code/ros/Launchers/Pod/Waypoints","Yaml Files (*.yaml)");
		if (!route.isEmpty()) {
			//waypoints_path_planner::routeinfo temp;
			//temp.index=route_info_array_.routeinfodata.size();
			temp.route_path = route.toStdString();
			route_info_array_.routeinfodata.push_back(temp);
			//routes_file_array_.push_back( route.toStdString());
			if (route_info_array_.routeinfodata.size() > 1) {
				index_spinbox_ -> setRange(0, route_info_array_.routeinfodata.size() - 1);
				index_spinbox_ -> setValue(route_info_array_.routeinfodata.size());
				std::string filename = route_info_array_.routeinfodata.at(route_info_array_.routeinfodata.size()-1).route_path;
				//std::string filename = routes_file_array_.at(routes_file_array_.size() - 1);
				segment_filename_lineedit_ -> setText(filename.c_str());
			}else if (route_info_array_.routeinfodata.size() == 1) {
				index_spinbox_ -> setRange(0, 0);
				index_spinbox_ -> setValue(0);
				std::string filename = route_info_array_.routeinfodata.at(0).route_path;
				segment_filename_lineedit_ -> setText(filename.c_str());
			}
			table_widget_->clear();
			table_widget_ = new QTableWidget(route_info_array_.routeinfodata.size(), route_info_array_.routeinfodata.size(), this);
			grid_layout_for_routes_manipulation_panel_->addWidget(table_widget_, 5,0,1,3);
			initialize_qtable_widget();
			matrix_map=read_data_from_qtable_to_unordered_map();
			save_route();
		}
		else
		{
			QMessageBox::information(this, "INFORMATION", "No Route Changes Done", QMessageBox::Ok);
		}
	}
	else
	{
		QMessageBox::information(this, "INFORMATION", "Only 4 routes are available for this version", QMessageBox::Ok);
	}
}
void WaypointsPathPlanner::delete_segment_qslot()
{
	if (route_info_array_.routeinfodata.size() > 1) {
		route_info_array_.routeinfodata.erase(std::find(route_info_array_.routeinfodata.begin(), route_info_array_.routeinfodata.end(),route_info_array_.routeinfodata.at(index_spinbox_->value())));
		//routes_file_array_.erase(std::find(routes_file_array_.begin(), routes_file_array_.end(),routes_file_array_.at(index_spinbox_->value())));
		index_spinbox_ -> setRange(0, route_info_array_.routeinfodata.size() - 1);
		index_spinbox_ -> setValue(route_info_array_.routeinfodata.size());
		std::string filename = route_info_array_.routeinfodata.at(route_info_array_.routeinfodata.size()-1).route_path;
		segment_filename_lineedit_ -> setText(filename.c_str());
		table_widget_->clear();
		table_widget_ = new QTableWidget(route_info_array_.routeinfodata.size(), route_info_array_.routeinfodata.size(), this);
		grid_layout_for_routes_manipulation_panel_->addWidget(table_widget_, 5,0,1,3);
		initialize_qtable_widget();

		/*table_widget_->removeColumn(index_spinbox_->value());
		table_widget_->removeRow(index_spinbox_->value());
		table_widget_->setHorizontalHeaderLabels(generate_node_names());
		table_widget_->setVerticalHeaderLabels(generate_node_names());
		table_widget_->setEditTriggers(QAbstractItemView::DoubleClicked);
		 */	matrix_map = read_data_from_qtable_to_unordered_map();
	} else if (route_info_array_.routeinfodata.size() == 1) {
		route_info_array_.routeinfodata.erase(std::find(route_info_array_.routeinfodata.begin(), route_info_array_.routeinfodata.end(),route_info_array_.routeinfodata.at(index_spinbox_->value())));
		index_spinbox_ -> setRange(0, 0);
		index_spinbox_ -> setValue(0);
		segment_filename_lineedit_ -> setText("");
		table_widget_->clear();
		table_widget_ = new QTableWidget(route_info_array_.routeinfodata.size(), route_info_array_.routeinfodata.size(), this);
		grid_layout_for_routes_manipulation_panel_->addWidget(table_widget_, 5,0,1,3);
		initialize_qtable_widget();

		/*table_widget_->removeColumn(index_spinbox_->value());
		table_widget_->removeRow(index_spinbox_->value());
		table_widget_->setHorizontalHeaderLabels(generate_node_names());
		table_widget_->setVerticalHeaderLabels(generate_node_names());
		table_widget_->setEditTriggers(QAbstractItemView::DoubleClicked);
		 */	matrix_map = read_data_from_qtable_to_unordered_map();
	} else {
		index_spinbox_ -> setRange(0, route_info_array_.routeinfodata.size());
		index_spinbox_ -> setValue(0);
		segment_filename_lineedit_ -> setText("");
		QMessageBox::information(this, "INFORMATION", "NO MORE ROUTES AVAILABLE", QMessageBox::Ok);
	}
	save_route();
}
void WaypointsPathPlanner::save_route_qslot()
{
	matrix_map = read_data_from_qtable_to_unordered_map();
	save_route();
	routes_manipulation_panel_-> close();
	routes_manipulation_panel_ = NULL;

}
QString WaypointsPathPlanner::generateColumnName(int index) {
	QString name;
	while (index >= 0) {
		name.prepend(QChar('A' + (index % 26)));  // Get the corresponding letter
		index = index / 26 - 1;  // Move to the next letter
	}
	return name;
}
QString WaypointsPathPlanner::generateRowName(int index) {
	QString name;
	while (index >= 0) {
		name.prepend(QChar('B' + (index % 26)));  // Get the corresponding letter
		index = index / 26 - 1;  // Move to the next letter
	}
	return name;
}
QStringList WaypointsPathPlanner::generate_node_names() {
	char letter = 'A';
	QStringList node_names;
	for (int i = 0; i < route_info_array_.routeinfodata.size(); ++i) {
		node_names.append(QString(letter).arg(i + 1));
		letter++;
	}
	return node_names;
}
void WaypointsPathPlanner::reset_matrix_qslot() {
	// Reset the matrix to "No"
	for (int row = 0; row < route_info_array_.routeinfodata.size(); ++row) {
		for (int col = 0; col < route_info_array_.routeinfodata.size(); ++col) {
			table_widget_->item(row, col)->setText("false");
		}
	}
}
void WaypointsPathPlanner::cell_clicked_qslot(int row, int col) {
	// Toggle cell value between "true" and "false"

	if (table_widget_->item(row, col)->text() == "false") {
		table_widget_->item(row, col)->setText("true");
		table_widget_->setEditTriggers(QAbstractItemView::DoubleClicked);
	} else {
		table_widget_->item(row, col)->setText("false");
		table_widget_->setEditTriggers(QAbstractItemView::DoubleClicked);
	}
}
void WaypointsPathPlanner::initialize_qtable_widget() {
	QStringList col_headers, row_headers;

	for (int i = 0; i < route_info_array_.routeinfodata.size(); ++i) {
		col_headers << generateColumnName(i);
		row_headers << generateRowName(i);
	}


	// Populate QTableWidget with the loaded data from unordered map

	table_widget_->setEditTriggers(QAbstractItemView::DoubleClicked);
	table_widget_->setHorizontalHeaderLabels(row_headers);
	table_widget_->setVerticalHeaderLabels(col_headers);
	table_widget_->setRowCount(route_info_array_.routeinfodata.size()); // Set row count based on map size
	table_widget_->setColumnCount(route_info_array_.routeinfodata.size());
	// Populate QTableWidget with the loaded data
	for (int row = 0; row < table_widget_->rowCount(); ++row) {
		//row_headers << generateRowName(row);
		for (int col = 0; col < table_widget_->columnCount(); ++col) {
			std::string row_label = table_widget_->verticalHeaderItem(row)->text().toStdString();
			std::string col_label = table_widget_->horizontalHeaderItem(col)->text().toStdString();
			//col_headers << generateColumnName(col);
			if (matrix_map.find({row_label, col_label}) != matrix_map.end()) {
				bool has_route = matrix_map[{row_label, col_label}];
				table_widget_->setItem(row, col, new QTableWidgetItem(has_route ? "true" : "false"));
				table_widget_->setEditTriggers(QAbstractItemView::DoubleClicked);
				connect(table_widget_, &QTableWidget::cellClicked, this, &WaypointsPathPlanner::cell_clicked_qslot);
			}
			else
			{
				table_widget_->setItem(row, col, new QTableWidgetItem("false"));
				table_widget_->setEditTriggers(QAbstractItemView::DoubleClicked);
				connect(table_widget_, &QTableWidget::cellClicked, this, &WaypointsPathPlanner::cell_clicked_qslot);
			}
		}
	}
	/*table_widget_->setEditTriggers(QAbstractItemView::DoubleClicked);
	table_widget_->setHorizontalHeaderLabels(row_headers);
	table_widget_->setVerticalHeaderLabels(col_headers);
	table_widget_->setRowCount(route_info_array_.routeinfodata.size()); // Set row count based on map size
	table_widget_->setColumnCount(route_info_array_.routeinfodata.size());
	 */
}
void WaypointsPathPlanner::set_route_file_qslot()
{

	if (routes_manipulation_panel_ == NULL) {
		routes_manipulation_panel_ = new QWidget();

		grid_layout_for_routes_manipulation_panel_ = new QGridLayout;
		grid_layout_for_routes_manipulation_panel_ -> setVerticalSpacing(0);
		grid_layout_for_routes_manipulation_panel_ -> setHorizontalSpacing(0);

		routes_filename_label_ = new QLabel("Routes Filename:");
		routes_filename_label_ -> setFont(QFont("FreeSans", 72));
		routes_filename_label_ -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
		routes_filename_label_ -> setVisible(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(routes_filename_label_, 0, 0, 1, 1);

		routes_filename_lineedit_ = new QLineEdit;
		routes_filename_lineedit_ -> setFixedSize(150, 30);
		routes_filename_lineedit_ -> setReadOnly(true);
		routes_filename_lineedit_ -> setText(routes_filename_.c_str());
		grid_layout_for_routes_manipulation_panel_ -> addWidget(routes_filename_lineedit_, 0,1,1,3);


		routes_index_label_ = new QLabel("Routes Index:");
		routes_index_label_ -> setFont(QFont("FreeSans", 72));
		routes_index_label_ -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
		routes_index_label_ -> setVisible(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(routes_index_label_, 1, 0, 1, 1);

		index_spinbox_ = new QSpinBox;
		//index_spinbox_->setEnabled(true);

		if (route_info_array_.routeinfodata.size() > 1) {
			index_spinbox_ -> setRange(0, route_info_array_.routeinfodata.size() - 1);
			index_spinbox_ -> setValue(index_spinbox_->value());

		} else if (route_info_array_.routeinfodata.size() == 1) {
			index_spinbox_ -> setRange(0, 0);
			index_spinbox_ -> setValue(0);

		} else {
			index_spinbox_ -> setRange(0, route_info_array_.routeinfodata.size());
			index_spinbox_ -> setValue(0);

		}
		index_spinbox_ -> setSingleStep(1);
		index_spinbox_ -> setValue(0);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(index_spinbox_, 1, 1, 1, 1);
		connect(index_spinbox_, SIGNAL(valueChanged(int)), this, SLOT(index_value_changed_qslot(int)));

		segment_filename_label_ = new QLabel("Segment Filename:");
		segment_filename_label_ -> setFont(QFont("FreeSans", 72));
		segment_filename_label_ -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
		segment_filename_label_ -> setVisible(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(segment_filename_label_, 2, 0, 1, 1);

		segment_filename_lineedit_ = new QLineEdit;
		if (route_info_array_.routeinfodata.size() > 1) {
			std::string filename = route_info_array_.routeinfodata.at(index_spinbox_->value()).route_path;
			segment_filename_lineedit_ -> setText(filename.c_str());
		} else if (route_info_array_.routeinfodata.size() == 1) {
			std::string filename = route_info_array_.routeinfodata.at(0).route_path;
			segment_filename_lineedit_ -> setText(filename.c_str());
		} else {
			segment_filename_lineedit_ -> setText("");
			QMessageBox::information(this, "INFORMATION", "NO MORE ROUTES AVAILABLE", QMessageBox::Ok);
		}

		grid_layout_for_routes_manipulation_panel_ -> addWidget(segment_filename_lineedit_, 2, 1, 1, 3);

		add_route_button_ = new QPushButton("ADD SEGMENT", this);
		add_route_button_ -> setEnabled(true);
		add_route_button_ -> setFixedSize(150, 30);
		add_route_button_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
		add_route_button_ -> setFont(QFont("FreeSans"));
		grid_layout_for_routes_manipulation_panel_ -> addWidget(add_route_button_, 4, 1, 1, 1);
		connect(add_route_button_, SIGNAL(released()), this, SLOT(add_route_qslot()));

		delete_route_button_ = new QPushButton("DELETE SEGMENT", this);
		delete_route_button_ -> setEnabled(true);
		delete_route_button_ -> setFixedSize(150, 30);
		delete_route_button_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
		delete_route_button_ -> setFont(QFont("FreeSans"));
		grid_layout_for_routes_manipulation_panel_ -> addWidget(delete_route_button_, 4, 2, 1, 1);
		connect(delete_route_button_, SIGNAL(released()), this, SLOT(delete_segment_qslot()));

		save_route_button_ = new QPushButton("CLOSE ROUTE PANEL", this);
		save_route_button_ -> setEnabled(true);
		save_route_button_ -> setFixedSize(150, 30);
		save_route_button_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
		save_route_button_ -> setFont(QFont("FreeSans"));
		grid_layout_for_routes_manipulation_panel_ -> addWidget(save_route_button_, 4, 3, 1, 1);
		connect(save_route_button_, SIGNAL(released()), this, SLOT(save_route_qslot()));

		table_widget_ = new QTableWidget(route_info_array_.routeinfodata.size(), route_info_array_.routeinfodata.size(), this);
		grid_layout_for_routes_manipulation_panel_->addWidget(table_widget_, 5,0,1,3);
		table_widget_->setEnabled(true);
		//table_widget_->setHorizontalHeaderLabels(generate_node_names());
		//table_widget_->setVerticalHeaderLabels(generate_node_names());
		initialize_qtable_widget();

		QPushButton *reset_button = new QPushButton("Reset Matrix", this);
		connect(reset_button, &QPushButton::clicked, this, &WaypointsPathPlanner::reset_matrix_qslot);
		grid_layout_for_routes_manipulation_panel_->addWidget(reset_button,5,3,1,1);

		/*route_possibility_ = new QLabel("CHECK ROUTE POSSIBILITY");
		route_possibility_ -> setFont(QFont("FreeSans", 72));
		route_possibility_ -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
		route_possibility_ -> setVisible(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(route_possibility_, 6, 1, 1, 1);
		 */
		origin_input_ = new QLabel("ORIGIN:");
		origin_input_ -> setFont(QFont("FreeSans", 72));
		origin_input_ -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
		origin_input_ -> setVisible(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(origin_input_, 8, 0, 1, 1);

		input_origin_ = new QLineEdit;
		input_origin_ -> setFixedSize(150, 30);
		//input_origin_ -> setReadOnly(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(input_origin_, 8, 1, 1, 1);

		destination_input_ = new QLabel("DESTINATION:");
		destination_input_ -> setFont(QFont("FreeSans", 72));
		destination_input_ -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
		destination_input_ -> setVisible(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(destination_input_, 8, 2, 1, 1);

		input_destination_ = new QLineEdit;
		input_destination_ -> setFixedSize(150, 30);
		//input_destination_ -> setReadOnly(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(input_destination_, 8, 3, 1, 1);

		check_route_possibility_ = new QPushButton("CHECK ROUTE POSSIBILITY", this);
		check_route_possibility_ -> setEnabled(true);
		check_route_possibility_ -> setFixedSize(175, 30);
		check_route_possibility_ -> setStyleSheet("font-weight: bold;color: black; font-size:9pt");
		check_route_possibility_ -> setFont(QFont("FreeSans"));
		grid_layout_for_routes_manipulation_panel_ -> addWidget(check_route_possibility_, 9, 1, 2, 2);
		connect(check_route_possibility_, SIGNAL(released()), this, SLOT(does_route_exist_qslot()));

		route_possibility_ = new QLabel("DOES ROUTE EXIST?");
		route_possibility_ -> setFont(QFont("FreeSans", 72));
		route_possibility_ -> setStyleSheet("font-weight: bold;color: black; font-size:11pt");
		route_possibility_ -> setVisible(true);
		grid_layout_for_routes_manipulation_panel_ -> addWidget(route_possibility_, 9, 3, 1, 1);

	}
	routes_manipulation_panel_ -> setFixedSize(700, 450);
	routes_manipulation_panel_ -> setLayout(grid_layout_for_routes_manipulation_panel_);
	routes_manipulation_panel_ -> setWindowTitle("Routes Management Panel");
	routes_manipulation_panel_ -> setWindowFlags(Qt::FramelessWindowHint);
	routes_manipulation_panel_ -> setWindowFlags(Qt::WindowTitleHint);
	routes_manipulation_panel_ -> show();
	routes_manipulation_panel_ -> activateWindow();



}
std::map<std::pair<std::string, std::string>, bool> WaypointsPathPlanner::read_matrix_from_yaml(const string& filename) {
	matrix_map.clear();
	try {
		YAML::Node yaml_data = YAML::LoadFile(filename);

		for (const auto& row : yaml_data["RoutesPossibilityMatrix"]) {
			std::string row_label = row.first.as<std::string>();

			for (const auto& col : row.second) {
				std::string col_label = col.first.as<std::string>();
				bool has_route = col.second.as<bool>();

				matrix_map[{row_label, col_label}] = has_route;
			}
		}
	} catch (const std::exception& e) {
		std::cerr << "Error loading YAML file: " << e.what() << std::endl;
	}
	return matrix_map;
}
std::map<std::pair<std::string, std::string>, bool>  WaypointsPathPlanner::read_data_from_qtable_to_unordered_map() {
	// Loop through the rows and columns of the QTableWidget
	matrix_map.clear();
	for (int row = 0; row < table_widget_->rowCount(); ++row) {
		for (int col = 0; col < table_widget_->columnCount(); ++col) {
			QTableWidgetItem* item = table_widget_->item(row, col);
			if (item) {
				std::string row_label = table_widget_->verticalHeaderItem(row)->text().toStdString();
				std::string col_label = table_widget_->horizontalHeaderItem(col)->text().toStdString();
				bool has_route = (item->text() == "true");  // Convert "Yes" to true, "No" to false
				matrix_map[{row_label, col_label}] = has_route;
			}
		}
	}

	return matrix_map;

}
bool WaypointsPathPlanner::does_route_exist_between_origin_destination(std::string origin, std::string destination) {
	auto it = matrix_map.find({origin, destination});
	if (it != matrix_map.end()) {
		return it->second; // Return true if a direct route exists
		//does_route_exist_ = it->second;
	}
	//std::cout<<"Does route exist"<<does_route_exist_<<std::endl;
}
void WaypointsPathPlanner::does_route_exist_qslot() {
	std::string origin, destination;
	origin = input_origin_->text().toStdString();
	destination = input_destination_->text().toStdString();
	auto it = matrix_map.find({origin, destination});
	if (it != matrix_map.end()) {
		//return it->second; // Return true if a direct route exists
		does_route_exist_ = it->second;
	}
	std::cout<<"Does route exist"<<does_route_exist_<<std::endl;
	if(does_route_exist_== true)
	{
		route_possibility_->setText("ROUTE EXIST");
	}
	else
	{
		route_possibility_->setText("ROUTE DOES NOT EXIST");
	}
}


void WaypointsPathPlanner::run_route_1_qslot()
{
	process_route_request(0);
}
void WaypointsPathPlanner::run_route_2_qslot()
{
	process_route_request(1);
}
void WaypointsPathPlanner::run_route_3_qslot()
{
	process_route_request(2);
}
void WaypointsPathPlanner::run_route_4_qslot()
{
	process_route_request(3);
}

void WaypointsPathPlanner::load_waypoints_file_qslot()
{
	ROS_INFO_STREAM("LoadWayPointsCallback_Panel");

	if (wppngzm.waypoint_manipulation_panel_ != nullptr) {
		wppngzm.waypoint_manipulation_panel_->close();
		wppngzm.waypoint_manipulation_panel_ = nullptr;
	}

	// Ask user to select the file
	QString selected_file = QFileDialog::getOpenFileName(this,
			tr("Select WayPoint File"),
			"/home/aurrigo-ads/code/ros/Launchers/Pod/3D_ACS_LAUNCHERS/LAUNCHERS/Waypoints",
			tr("SEGMENT FILE (*.yaml);;All Files (*)"));

	if (selected_file.isEmpty()) {
		return; // User canceled
	}

	// Store the filename
	filename_ = selected_file.toStdString();
	//std::string filepath = filename_;

	boost::filesystem::path filename_path(filename_);
	left_lane_filename_ = filename_path.parent_path().string() + "/LeftLane/leftlane.yaml";
	right_lane_filename_ = filename_path.parent_path().string() + "/RightLane/rightlane.yaml";

	publish_waypoint_paths();

	wppngzm.clear_waypoints_descriptors_arrays();
	wppngzm.clear_no_go_zone_objects_before_loading_file();

	ROS_INFO_STREAM("New File Loaded: " << filename_);
	QMessageBox::information(this, "INFORMATION", "NEW FILE LOADED", QMessageBox::Ok);

	// Load data
	wppngzm.load_waypoints_and_nogozones(filename_, left_lane_filename_, right_lane_filename_);
	wppngzm.load_route_process(filename_, left_lane_filename_, right_lane_filename_);
}

void WaypointsPathPlanner::clearall_waypoint_markers_qslot()
{
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this, "Clear All Markers ", "DO YOU WANT TO CLEAR ALL WAYPOINTS FROM FILE?",
			QMessageBox::Yes | QMessageBox::No);
	if (reply == QMessageBox::Yes) {
		qDebug() << "Yes was clicked";
		if (wppngzm.waypoint_manipulation_panel_ != NULL) {
			wppngzm.waypoint_manipulation_panel_ -> close();
			wppngzm.waypoint_manipulation_panel_ = NULL;
			set_button_state(create_waypoint_file_,true);
			set_button_state(load_waypoints_file_,true);
			set_button_state(start_navigation_,false);
		}
		wppngzm.clear_waypoints_descriptors_arrays();
		wppngzm.save_waypoints_and_nogozones();
		does_waypoints_exist_in_left_lane_ = false;
		does_waypoints_exist_in_main_lane_ = false;
		does_waypoints_exist_in_right_lane_ = false;
	} else {
		qDebug() << "Yes was *not* clicked";
	}
	ROS_INFO_STREAM("clearMarkersCallback_Panel");
}

void WaypointsPathPlanner::compute_segment_qslot()
{
	//wppngzm.compute_segment_process(wppfm.filename_, wppfm.left_lane_filename_, wppfm.right_lane_filename_);
	// Check if files are set and exist
	if (filename_.empty()) {
		qWarning() << "Main waypoint file is not set.";
		return;
	}

	// Possibly also check left/right lanes only if they're used in routing
	// Log or inform the GUI
	qDebug() << "Starting route computation...";

	// Call compute
	wppngzm.compute_segment_process(filename_,left_lane_filename_,right_lane_filename_);

	qDebug() << "Route computation complete.";
}
void WaypointsPathPlanner::pause_release_qslot()
{
	std_msgs::Int32 destination;
	destination.data = 1;
	pause_release_publisher_.publish(destination);
	debug_message_edit_ -> clear();
	debug_message_edit_ -> setText("Pause released");
}

void WaypointsPathPlanner::delete_no_go_zone_markers_in_file_and_rviz_qslot()
{
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this, "Clear No Go Zones ", "DO YOU WANT TO CLEAR ALL NO GO ZONES FROM FILE?",
			QMessageBox::Yes | QMessageBox::No);
	if (reply == QMessageBox::Yes) {
		qDebug() << "Yes was clicked";
		wppngzm.clear_no_go_zone_objects_before_loading_file();
		wppngzm.save_waypoints_and_nogozones();
		} else {
		qDebug() << "Yes was *not* clicked";
	}
	ROS_INFO_STREAM("clear no go zone markers !!!!!!");
}
/* Subscriber's Callback Definitions*/
void WaypointsPathPlanner::dynamic_alignment_callback(waypoints_path_planner::waypointinfo msg) {
	update_waypoint_dynamically(msg.index, msg.waypointpose.position.x, msg.waypointpose.position.y);
	start_navigation_->click();
}
void WaypointsPathPlanner::black_box_recorder_callback(VehicleBlackboxRecorderMsg msg) {
	std::cout << "Im in Callback BBR" << std::endl;
	timer_for_bbr_status_.start();
	time_from_bbr_status_ = ros::Time::now();

	boot_safe = msg.BootSafe;

	if ((msg.BlackBoxSystemState == 0) && (msg.PercentFull <= 25)) {
		label_bbr_status_ -> setPixmap(bbr_ok_0_);
		label_bbr_status_ -> show();
	} else if ((msg.BlackBoxSystemState == 0) && ((msg.PercentFull > 25) && (msg.PercentFull <= 50))) {
		label_bbr_status_ -> setPixmap(bbr_ok_2_);
		label_bbr_status_ -> show();
	} else if ((msg.BlackBoxSystemState == 0) && ((msg.PercentFull > 50) && (msg.PercentFull <= 75))) {
		label_bbr_status_ -> setPixmap(bbr_ok_3_);
		label_bbr_status_ -> show();
	} else if ((msg.BlackBoxSystemState == 0) && ((msg.PercentFull > 75) && (msg.PercentFull <= 95))) {
		label_bbr_status_ -> setPixmap(bbr_ok_4_);
		label_bbr_status_ -> show();
	} else if ((msg.BlackBoxSystemState == 4) || (msg.BlackBoxSystemState == 7)) {
		label_bbr_status_ -> setPixmap(bbr_stop_);
		label_bbr_status_ -> show();
	} else if (msg.BlackBoxSystemState == 5) {
		label_bbr_status_ -> setMovie(movie_bbr_no_drive_in_bay_);
		movie_bbr_no_drive_in_bay_ -> start();

	} else if (msg.BlackBoxSystemState == 6) {
		label_bbr_status_ -> setMovie(movie_bbr_full_);
		movie_bbr_full_ -> start();
	}

}
void WaypointsPathPlanner::av_to_ads_aux_callback(VehicleAVToADSAUXMsg msg) {
	timer_for_battery_status_.start();
	time_from_the_pod_to_acs_aux_ = ros::Time::now();
	if ((pod_state_msg_ == (uint8_t) 13) && (msg.StateOfCharge < 95)) {
		label_battery_default_ -> setMovie(movie_battery_charging_);
		movie_battery_charging_ -> start();
	} else if ((msg.StateOfCharge >= 95) && (pod_state_msg_ == (uint8_t) 13)) {
		label_battery_default_ -> setMovie(movie_battery_full_plugged_in_);
		movie_battery_full_plugged_in_ -> start();
	} else if (msg.StateOfCharge >= 76) {
		label_battery_default_ -> setMovie(movie_battery_full_);
		movie_battery_full_ -> start();
	} else if ((msg.StateOfCharge > 50) && (msg.StateOfCharge <= 75)) {
		label_battery_default_ -> setMovie(movie_battery_three_quarter_);
		movie_battery_three_quarter_ -> start();
	} else if ((msg.StateOfCharge > 25) && (msg.StateOfCharge <= 50)) {
		label_battery_default_ -> setMovie(movie_battery_half_);
		movie_battery_half_ -> start();
	} else if ((msg.StateOfCharge > 5) && (msg.StateOfCharge <= 25)) {
		label_battery_default_ -> setMovie(movie_battery_quarter_);
		movie_battery_quarter_ -> start();
	} else if (msg.StateOfCharge <= 5) {
		label_battery_default_ -> setMovie(movie_battery_depleated_);
		movie_battery_depleated_ -> start();
	}
	if (msg.AutonomyEnableSwitchState == true) {
		label_autonomy_enable_disable_ -> setMovie(movie_autonomy_enable_on_);
		movie_autonomy_enable_on_ -> start();
	} else if (msg.AutonomyEnableSwitchState == false) {
		label_autonomy_enable_disable_ -> setMovie(movie_autonomy_enable_off_);
		movie_autonomy_enable_off_ -> start();
	}
}
void WaypointsPathPlanner::pod_to_acs_core_callback(VehicleAVToADSCOREMsg msg) {
	timer_for_battery_status_.start();
	time_from_the_pod_to_acs_core_ = ros::Time::now();
	//if(msg.PodState==(uint8_t)13)
	//	{
	//		label_battery_default_->setMovie(movie_battery_charging_);
	//		movie_battery_charging_->start();
	//	}
	pod_state_msg_ = (uint8_t) msg.AvState;
}
void WaypointsPathPlanner::time_machine_callback(VehicleTimeMachineMsg msg) {
	timer_for_time_machine_status_.start();
	time_from_time_machine_status_ = ros::Time::now();
	if (msg.TimeMachineSystemState == (uint8_t) 0) {
		label_time_machine_ -> setMovie(movie_time_machine_ready_);
		movie_time_machine_ready_ -> start();
	} else if (msg.TimeMachineSystemState == (uint8_t) 2) {
		label_time_machine_ -> setMovie(movie_time_machine_writing_);
		movie_time_machine_writing_ -> start();
	} else {
		label_time_machine_ -> setPixmap(time_machine_error_);
		label_time_machine_ -> show();
	}
}
void WaypointsPathPlanner::system_monitor_callback(VehicleSystemMonitorMsg msg) {
	timer_for_system_monitor_status_.start();
	time_from_system_monitor_status_ = ros::Time::now();
	if ((msg.GPUTemperature > 85) && (msg.CPU0TempCurrent > 70)) {
		label_system_monitor_ -> setPixmap(cpu_temp_high_);
	} else if (msg.CPU0TempCurrent > 70) {
		label_system_monitor_ -> setPixmap(cpu_temp_high_);
	} else if (msg.GPUTemperature > 85) {
		label_system_monitor_ -> setPixmap(gpu_temp_high_);
	} else {
		label_system_monitor_ -> setPixmap(ads_temp_ok_);
	}
}
void WaypointsPathPlanner::merged_cloud_callback(sensor_msgs::PointCloud2 msg) {
	timer_for_lidar_status_on_off_.start();
	time_from_lidar_status_ = ros::Time::now();
	label_lidar_status_ -> setPixmap(lidar_status_on_);
	label_lidar_status_ -> show();
}
void WaypointsPathPlanner::micro_imu_callback(sensor_msgs::Imu msg) {
	timer_for_imu_fix_on_off_.start();
	time_from_imu_status_ = ros::Time::now();
	label_imu_fix_on_off_ -> setMovie(movie_imu_fix_on_);
	movie_imu_fix_on_ -> start();
}
void WaypointsPathPlanner::dms_status_callback(VehicleDMSStatusMsg msg) {
	timer_for_dms_handle_status_.start();
	time_from_dms_status_ = ros::Time::now();
	if (msg.state == (uint8_t) 0) {
		label_dms_on_off_ -> setMovie(movie_dms_on_off_uninitialised_);
		movie_dms_on_off_uninitialised_ -> start();
	}
	/*else if(msg.state>(uint8_t)0)
    {
    	label_dms_on_off_->setMovie(movie_dms_off_);
    	movie_dms_off_->start();
    }*/
	else if (msg.state > (uint8_t) 0) {
		label_dms_on_off_ -> setMovie(movie_dms_on_);
		movie_dms_on_ -> start();
	}
}
void WaypointsPathPlanner::av_stop_request_callback(std_msgs::Bool stop)
{
	wpplm.cancel_av_navigation_process();
}
void WaypointsPathPlanner::localization_pose_callback(geometry_msgs::PoseWithCovarianceStamped pose) {
	check_localization_pose_ = true;
	geometry_msgs::Pose temp_pose;

	adt_x = pose.pose.pose.position.x;
	adt_y = pose.pose.pose.position.y;
	temp_pose.position.x = vehicle_pose_.pose.position.x = pose.pose.pose.position.x;
	temp_pose.position.y =vehicle_pose_.pose.position.y = pose.pose.pose.position.y;
	temp_pose.position.z =vehicle_pose_.pose.position.z = 0;
	temp_pose.orientation.z = vehicle_pose_.pose.orientation.z = pose.pose.pose.orientation.z;
	temp_pose.orientation.w =vehicle_pose_.pose.orientation.w = pose.pose.pose.orientation.w;
	temp_pose.orientation.x =vehicle_pose_.pose.orientation.x = 0;
	temp_pose.orientation.y =vehicle_pose_.pose.orientation.y = 0;
	distance_from_home_ = wpplm.calculate_distance_between_two_points(home_pose_, temp_pose);
}
void WaypointsPathPlanner::gps_pod_pose_callback(nav_msgs::Odometry gps_pose) {
	check_gps_pod_pose_ = true;
	geometry_msgs::Pose temp_pose;
	temp_pose.position.x = vehicle_pose_from_gps_.pose.position.x = gps_pose.pose.pose.position.x;
	temp_pose.position.y = vehicle_pose_from_gps_.pose.position.y = gps_pose.pose.pose.position.y;
	temp_pose.position.z = vehicle_pose_from_gps_.pose.position.z = 0;
	temp_pose.orientation.x = vehicle_pose_from_gps_.pose.orientation.x = 0;
	temp_pose.orientation.y = vehicle_pose_from_gps_.pose.orientation.y = 0;
	temp_pose.orientation.z = vehicle_pose_from_gps_.pose.orientation.z = gps_pose.pose.pose.orientation.z;
	temp_pose.orientation.w = vehicle_pose_from_gps_.pose.orientation.w = gps_pose.pose.pose.orientation.w;
	distance_from_home_ = wpplm.calculate_distance_between_two_points(home_pose_, temp_pose);
}
void WaypointsPathPlanner::is_av_navigating_callback(std_msgs::Bool msg) {
	timer_for_navigation_status_.start();
	time_from_navigation_status_ = ros::Time::now();
	is_av_navigating_msg_.data = msg.data;
	if (is_av_navigating_msg_.data == true) {
		navigation_image_label_ -> setPixmap(navigation_running_);
	} else if (is_av_navigating_msg_.data == false) {
		navigation_image_label_ -> setPixmap(navigation_stopped_);
	} else if ((is_av_navigating_msg_.data == false) && (pause_set_.data == true)) {
		navigation_image_label_ -> setPixmap(navigation_paused_);
	}
}
void WaypointsPathPlanner::current_vehiclepose_from_localplanner_callback(waypoints_path_planner::currentpodposewithtrajectoryln msg){
	line_number_of_vehicle_pose_in_the_global_plan_ = msg.linenumber; //variable changed
	current_vehicle_pose_from_local_planner_ = msg.pose;
}
void WaypointsPathPlanner::detailed_route_from_globalplanner_callback(const std_msgs::String::ConstPtr & msg){
	if (msg != NULL) {
		is_filepath_received = true; //variable changed
		file_location_of_vehicle_global_plan = msg -> data.c_str(); //variable changed used in lane switch function
	}
}
void WaypointsPathPlanner::is_local_goal_reached_callback(std_msgs::Bool msg) {
	is_local_goal_reached_.data = msg.data;
	is_goal_reached_ = msg.data;
}
void WaypointsPathPlanner::pause_set_callback(std_msgs::Bool msg) {
	pause_set_.data = msg.data;
}
#ifdef VEH_TYPE_POD
void WaypointsPathPlanner::pause_release_from_controller_callback(VehicleAVToADSControllerMsg msg)
{
	xbox_buttons_ = msg.XboxButtons;
	// pod_state_ = msg.AvState; // SPB pod_state is not available here as it is in av_to_ads_core msg
	if ((old_button_state_ != xbox_buttons_) && (pod_state_msg_ == 11)) //if((old_button_state_==0)&&(xbox_buttons_==16384) && (pod_state_==11))
	{
		std_msgs::Int32 destination;

		if (xbox_buttons_ == 16384) {
			destination.data = 1;
		} else {
			destination.data = 0;
		}
		pause_release_publisher_.publish(destination);
		debug_message_edit_ -> clear();
		debug_message_edit_ -> setText("Pause Released");
	} else {
		// do nothing
	}
	old_button_state_ = xbox_buttons_;
}
#endif
void WaypointsPathPlanner::plot_no_go_zone_callback(const geometry_msgs::PointStamped & msg) {

	if (is_rviz_locked_unlocked_.data == false) {
		bool ok;
		double radius_of_Circle = QInputDialog::getDouble(this, tr("Enter Radius "), tr("Radius"), 0.0, 0, 10, 2, & ok);
		QMessageBox::StandardButton direction_flag;
		direction_flag = QMessageBox::question(this, "NGZ direction", "ADT stops if obstacle is moving in direction", QMessageBox::No | QMessageBox::Yes | QMessageBox::Ignore);
		int ngz_version;
		if (ok && radius_of_Circle != 0) {
			radius_of_the_no_go_zone_ = radius_of_Circle;
			visualization_msgs::InteractiveMarker no_go_zone_marker;
			no_go_zone_marker.pose.position.x = msg.point.x;
			no_go_zone_marker.pose.position.y = msg.point.y;
			no_go_zone_marker.pose.position.z = msg.point.z;
			no_go_zone_marker.pose.orientation.x = 0;
			no_go_zone_marker.pose.orientation.y = 0;
			no_go_zone_marker.pose.orientation.z = 0;
			no_go_zone_marker.pose.orientation.w = 1;
			if (direction_flag == QMessageBox::Yes) //slows down if obstacle moves in direction
			{
				ngz_version = 1;
			}
			else if (direction_flag == QMessageBox::No) //slows down if obstacle moves in any other direction
			{
				ngz_version = 2;
			}
			else //slows down if obstacle is within ngz
			{
				ngz_version = 0;
			}
			wppngzm.create_no_go_zone_markers(no_go_zone_marker,radius_of_the_no_go_zone_,ngz_version);
			wppngzm.save_waypoints_and_nogozones();
			wppngzm.clear_no_go_zone_objects_before_loading_file();
			wppngzm.clear_waypoints_descriptors_arrays();
			wppngzm.load_waypoints_and_nogozones(filename_,left_lane_filename_,right_lane_filename_);
		} else {
			QMessageBox::information(this, "INFORMATION", "Enter a radius greater than zero", QMessageBox::Ok);
		}
	} else {
		QMessageBox::information(this, "INFORMATION", "UNLOCK RVIZ USING THE LOCK/UNLOCK BUTTON TO PLOT NEW NO GO ZONES", QMessageBox::Ok);
	}
	no_go_zone_position_++;
}
void WaypointsPathPlanner::plot_waypoints_2d_nav_goal(geometry_msgs::PoseStamped msg) {
	if (is_rviz_locked_unlocked_.data) {
		QMessageBox::information(this, "INFORMATION", "UNLOCK RVIZ USING THE LOCK/UNLOCK BUTTON TO PLOT NEW WAYPOINTS", QMessageBox::Ok);
		return;
	}

	if (is_av_navigating_msg_.data) {
		QMessageBox::information(this, "CAUTION", "POD NAVIGATING!!! WAIT FOR THE ROUTE TO COMPLETE. CANNOT CLEAR WAYPOINTS WHILE POD IS NAVIGATING!", QMessageBox::Ok);
		return;
	}

	if (counter_for_2d_nav_goal_ == 0) {
		QMessageBox::information(this, "CAUTION",
			"OVERRIDING 2D NAV GOAL FOR WAYPOINT PLOTTING CAUSES MOVEBASE TO CRASH.\n"
			"CLOSE RVIZ AFTER ALL WAYPOINTS ARE PLOTTED AND THEN START NAVIGATION IN RVIZ AGAIN!",
			QMessageBox::Ok);
	}

	counter_for_2d_nav_goal_++;
	position_ = 0;

	visualization_msgs::InteractiveMarker waypoint_marker;
	waypoint_marker.pose.position = msg.pose.position;
	waypoint_marker.pose.orientation = msg.pose.orientation;

	if (wppngzm.waypoint_manipulation_panel_ != nullptr) {
		wppngzm.save_button_->click();
	}

	// Create markers for all lanes
	wppngzm.create_waypoint_markers(waypoint_marker, "main_lane", "main_lane_waypoint", -1, position_, 0);
	wppngzm.create_waypoint_markers(waypoint_marker, "left_lane", "left_lane_waypoint", -1, position_, 0);
	wppngzm.create_waypoint_markers(waypoint_marker, "right_lane", "right_lane_waypoint", -1, position_, 0);

	// Save and reload updated waypoints
	wppngzm.save_waypoints_and_nogozones();
	wppngzm.clear_no_go_zone_objects_before_loading_file();
	wppngzm.clear_waypoints_descriptors_arrays();
	wppngzm.load_waypoints_and_nogozones(filename_, left_lane_filename_, right_lane_filename_);

	position_++;

	// GUI feedback
	debug_message_edit_->clear();
	debug_message_edit_->setText("The waypoint is successfully placed in RVIZ and saved in the segment file");

	// Update panel spinbox ranges if available
	if (wppngzm.waypoint_manipulation_panel_ != nullptr) {
		int marker_count = static_cast<int>(wppngzm.main_lane_marker_names.size());

		if (marker_count > 0) {
			wppngzm.index_spinbox_->setRange(0, marker_count - 1);
			wppngzm.from_waypoint_spinbox_->setRange(0, marker_count - 1);
			wppngzm.to_waypoint_spinbox_->setRange(0, marker_count - 1);
		} else {
			// Auto-close the panel if nothing remains
			wppngzm.waypoint_manipulation_panel_->close();
			wppngzm.waypoint_manipulation_panel_ = nullptr;
		}
	}
}


/*Subscriber's with Timer Callback Definition*/

void WaypointsPathPlanner::vehicle_position_error_callback(std_msgs::Float32 msg) {
	timer_for_vehicle_position_error_status_.start();
	time_from_vehicle_position_error_status_ = ros::Time::now();
	if (msg.data <= 1.0) {
		covariance_image_label_ -> setPixmap(covariance_good_);
	} else if (msg.data >= 3.0) {
		covariance_image_label_ -> setPixmap(covariance_error_);
	} else if ((msg.data > 1.0) && (msg.data < 3)) {
		covariance_image_label_ -> setPixmap(covariance_poor_);
	}
}
void WaypointsPathPlanner::current_waypoint_index_callback(std_msgs::Int32 msg)
{
	current_waypoint_index_ = msg.data;
	if (current_waypoint_index_ > 0 && current_waypoint_index_ < 100) //index changes to 0 while in temp lane
	{
		ROS_INFO("LANE SWITCH RESETS");
		set_button_state(left_lane_switch_,true);
		set_button_state(right_lane_switch_,true);
		is_clicked = false;
	}

}
void WaypointsPathPlanner::global_map_path_callback(std_msgs::String msg)
{
	map_file_path_msg = msg;
}
void WaypointsPathPlanner::end_of_route_subscriber_callback(std_msgs::Bool msg)
{
	std::cout<<"End of route acheived"<<std::endl;
	if(command_based_filenames_to_load.size()!=0)
	{
		command_based_filenames_to_load.pop();
		end_of_route_=msg.data;
		//cancel_navigation_->click();

		if((!command_based_filenames_to_load.empty()) && end_of_route_==true )
		{
			//cancel_navigation_->click();
			command_load_segments(command_based_filenames_to_load);
			//start_navigation_->click();
		}
	}
}
void WaypointsPathPlanner::local_goal_reached_subscriber_callback(std_msgs::Bool msg)
{
	local_goal_reached_=msg.data;
}

void WaypointsPathPlanner::ipe_to_ads_callback(std_msgs::String msg)
{
	try
        {
            ROS_INFO("Received command: %s", msg.data.c_str());
            std::string received_message = msg.data;
            std::string first_two = received_message.substr(0, 2);
            std::string second_two = received_message.substr(2, 2);
            std::string first_four = received_message.substr(0, 4);
            std_msgs::String response_msg;
            std::string response;
 
            if (received_message.length() < 9)
            {
                switch (first_two[0])
                {
                case 'B':
                    switch (first_two[1])
                    {
                    case 'C':
                        cancel_navigation_qslot();
                        response = "bc--";
                        break;
                    case 'S':
                        start_av_navigation_qslot();
                        response = "bs--";
                        break;
                    default:
                        response = "nil_";
                        break;
                    }
                    break;
                case 'D':
                    if (first_two == "DR")
                    {
                        if (second_two == "00")
                        {
                            response = "dr00";
                        }
                        else if (second_two == "01")
                        {
                            set_button_state(run_route_1_file_, false);
                            response = "dr01";
                        }
                        else if (second_two == "02")
                        {
                            set_button_state(run_route_2_file_, false);
                            response = "dr02";
                        }
                        else if (second_two == "03")
                        {
                            set_button_state(run_route_3_file_, false);
                            response = "dr03";
                        }
                        else if (second_two == "04")
                        {
                            set_button_state(run_route_4_file_, false);
                            response = "dr04";
                        }
                        else
                        {
                            response = "nil_";
                        }
                    }
                    break;
                case 'E':
                    switch (first_two[1])
                    {
                    case 'R':
                        if (second_two == "00")
                        {
                            response = "er00";
                        }
                        else if (second_two == "01")
                        {
                            set_button_state(run_route_1_file_, true);
                            response = "er01";
                        }
                        else if (second_two == "02")
                        {
                            set_button_state(run_route_2_file_, true);
                            response = "er02";
                        }
                        else if (second_two == "03")
                        {
                            set_button_state(run_route_3_file_, true);
                            response = "er03";
                        }
                        else if (second_two == "04")
                        {
                            set_button_state(run_route_4_file_, true);
                            response = "er04";
                        }
                        else
                        {
                            response = "nil_";
                        }
                        break;
                    case 'V':
                        if (second_two == "PR" ||
                            second_two == "NO")
                        {
                            response = "ack_";
                        }
                        else
                        {
                            response = "nil_";
                        }
                        break;
                    default:
                        response = "nil_";
                        break;
                    }
                    break;

                case 'L':
                    switch (first_two[1])
                    {
                    case 'R':
                        response = "nil_";
                        break;
                    case 'S':
                    	cancel_navigation_qslot();
                        if (second_two == "00")
                        {
                            response = "ls00";
                        }
                        else if (second_two == "01")
                        {
                            run_route_1_file_->click();
                            response = "ls01";
                        }
                        else if (second_two == "02")
                        {
                            run_route_2_file_->click();
                            response = "ls02";
                        }
                        else if (second_two == "03")
                        {
                            run_route_3_file_->click();
                            response = "ls03";
                        }
                        else if (second_two == "04")
                        {
                            run_route_4_file_->click();
                            response = "ls04";
                        }
                        else
                        {
                            response = "nil_";
                        }
                        break;
                    default:
                        response = "nil_";
                        break;
                    }
                    break;

                case 'R':
                    switch (first_two[1])
                    {
                    case 'E':
                        response = "ack";
                        break;
                    case 'M':
                        response = "rm--";
                        break;
                    case 'N':
                        response = is_av_navigating_msg_.data ? "rn1-" : "rn0-";
                        break;
                    case 'O':
                        response = "ack";
                        break;
                    case 'R':
                        response = "rr--";
                        break;
                    case 'S':
                        response = "rs--";
                        break;
                    case 'T':
                        response = "rt--";
                        break;
                    case 'W':
                    {
                        float x_diff = adt_x - end_point_.position.x;
                        float y_diff = adt_y - end_point_.position.y;
                        float total_diff = sqrt(x_diff * x_diff + y_diff * y_diff);
                        try
                        {
                            if (total_diff <= 1.7)
                            {
                                response = "rw@@";
                            }
                            else
                            {
                                response = "rw" + int_to_hexstring(current_waypoint_index_);
                            }
                        }
                        catch (const std::exception &e)
                        {
                            response = "rw!!";
                        }
                        break;
                    }

                    default:
                        response = "nil_";
                        break;
                    }
                    break;

                case 'Z':
                    response = "ack_";
                    break;
                default:
                    response = "nil_";
                    break;
                }
            }
            else
            {
                response = "nil_";
            }
 
            response_msg.data = response;

            if(response=="nil_")
            {
                response_msg.data = last_minder_state_;
            } else {
                response_msg.data = response;
            }
            ads_to_ipe_publisher_.publish(response_msg);
        }

        catch (...)
        {
            ROS_INFO("done");
        }	
}

void WaypointsPathPlanner::lane_switch_callback(std_msgs::Bool msg)
{
	bool go_right_=msg.data;
	if(!is_clicked)
	{
		if(go_right_)
		{
			ROS_INFO("GONNA CLICK RIGHT");
			right_lane_switch_ -> click();
			ROS_INFO("RIGHT CLICKED");
		}
		else
		{
			left_lane_switch_ -> click();
			ROS_INFO("LEFT CLICKED");
		}
		is_clicked = true;
		ROS_INFO("SET TO TRUE");
	}

}

void WaypointsPathPlanner::minder_state_callback(std_msgs::String msg)
{
	last_minder_state_ = msg.data;  // Store latest received state
}

void WaypointsPathPlanner::waypoint_info_callback(waypoints_path_planner::waypointinfoarray msg)
{
	for (int i = 0; i < msg.waypointinfodata.size(); i++)
	{
		end_point_ = msg.waypointinfodata.at(msg.waypointinfodata.size() - 1).waypointpose;
	}
}

void WaypointsPathPlanner::command_load_segments(std::queue<std::string> command_based_filenames_to_load)
{
	if(!command_based_filenames_to_load.empty())
	{
		filename_=command_based_filenames_to_load.front();
		std::cout<<"File name in command_load_segments"<<filename_<<command_based_filenames_to_load.front()<<std::endl;
		boost::filesystem::path filename_path(command_based_filenames_to_load.front());
		left_lane_filename_ = filename_path.parent_path().string() + "/LeftLane" + "/leftlane.yaml";
		right_lane_filename_ = filename_path.parent_path().string() + "/RightLane" + "/rightlane.yaml";
		publish_waypoint_paths();
		wppngzm.clear_waypoints_descriptors_arrays();
		wppngzm.clear_no_go_zone_objects_before_loading_file();
		wppngzm.load_waypoints_and_nogozones(filename_, left_lane_filename_, right_lane_filename_);
		wppngzm.load_route_process(filename_, left_lane_filename_, right_lane_filename_);
		//if(is_av_navigating_msg_.data==false)
		{
			start_navigation_->click();
		}
		//command_based_filenames_to_load.pop();
	}

}

std::vector<std::string> WaypointsPathPlanner::segment_path_string(const std::string& input) {
	std::vector<std::string> segments;
	for (size_t i = 0; i < input.size() - 1; ++i) {
		segments.push_back(input.substr(i, 2)); // Extract 2-character substring
	}
	return segments;
}

/*Timer's Callback Definitions*/
void WaypointsPathPlanner::timer_for_vehicle_position_error() {
	last_time_in_timer_vehicle_position_error_status_ = time_from_vehicle_position_error_status_;
	duration_between_last_time_and_time_from_vehicle_position_error_status_ = ros::Time::now();
	ros::Duration final_time_for_vehicle_position_error_status = duration_between_last_time_and_time_from_vehicle_position_error_status_ - last_time_in_timer_vehicle_position_error_status_;
	if (final_time_for_vehicle_position_error_status.sec > 1) {
		//covariance_image_label_ -> setPixmap(covariance_unknown_);
		//covariance_image_label_ -> show();
		timer_for_vehicle_position_error_status_.stop();
	}
}
void WaypointsPathPlanner::timer_for_navigation_callback() {
	last_time_in_timer_navigation_status_ = time_from_navigation_status_;
	duration_between_last_time_and_time_from_navigation_status_ = ros::Time::now();
	ros::Duration final_time_for_navigation_status = duration_between_last_time_and_time_from_navigation_status_ - last_time_in_timer_navigation_status_;
	if (final_time_for_navigation_status.sec > 1) {
		navigation_image_label_ -> setPixmap(navigation_unknown_);
		navigation_image_label_ -> show();
		timer_for_navigation_status_.stop();
	}
	std_msgs::Int32 destination;
	destination.data = 0;
	pause_release_publisher_.publish(destination);
}
void WaypointsPathPlanner::timer_for_time_machine_callback() {
	last_time_in_timer_time_machine_status_ = time_from_time_machine_status_;
	duration_between_last_time_and_time_from_time_machine_status_ = ros::Time::now();
	ros::Duration final_time_for_time_machine_status = duration_between_last_time_and_time_from_time_machine_status_ - last_time_in_timer_time_machine_status_;
	if (final_time_for_time_machine_status.sec > 1) {
		label_time_machine_ -> setPixmap(time_machine_error_);
		label_time_machine_ -> show();
		timer_for_time_machine_status_.stop();
	}
}
void WaypointsPathPlanner::timer_for_system_monitor_callback() {
	last_time_in_timer_system_monitor_status_ = time_from_system_monitor_status_;
	duration_between_last_time_and_time_from_system_monitor_status_ = ros::Time::now();
	ros::Duration final_time_for_system_monitor_status = duration_between_last_time_and_time_from_system_monitor_status_ - last_time_in_timer_system_monitor_status_;
	if (final_time_for_system_monitor_status.sec > 1) {
		label_system_monitor_ -> setPixmap(ads_temp_unknown_);
		label_system_monitor_ -> show();
		timer_for_system_monitor_status_.stop();
	}
}
void WaypointsPathPlanner::timer_for_loading_arms_callback() {
	last_time_in_timer_loading_arms_status_ = time_from_loading_arms_status_;
	duration_between_last_time_and_time_from_loading_arms_status_ = ros::Time::now();
	ros::Duration final_time_for_loading_arms_status = duration_between_last_time_and_time_from_loading_arms_status_ - last_time_in_timer_loading_arms_status_;
	if (final_time_for_loading_arms_status.sec > 1) {
		label_loading_arms_ -> setPixmap(loading_arms_unknown_);
		label_loading_arms_ -> show();
		timer_for_loading_arms_status_.stop();
	}
}
// timer for gps_status
/*void InteractiveWaypoints::timerforGPSfixCallback()
  {
  	last_time_in_timer_gps_status_=time_from_gps_status_;
  	duration_between_last_time_and_time_from_gps_status_=ros::Time::now();
  	ros::Duration final_time_for_gps_status=duration_between_last_time_and_time_from_gps_status_-last_time_in_timer_gps_status_;
  	if(final_time_for_gps_status.sec>1)
  	{
  		label_gps_fix_on_off_->setPixmap(pic_gps_fix_off_);
  		label_gps_fix_on_off_->show();
  	}
  	else
  	{
  		label_gps_fix_on_off_->setPixmap(pic_gps_fix_on_);
  		label_gps_fix_on_off_->show();
  	}
  }*/
void WaypointsPathPlanner::timer_for_imu_callback() {
	last_time_in_timer_imu_status_ = time_from_imu_status_;
	duration_between_last_time_and_time_from_imu_status_ = ros::Time::now();
	ros::Duration final_time_for_imu_status = duration_between_last_time_and_time_from_imu_status_ - last_time_in_timer_imu_status_;
	if (final_time_for_imu_status.sec > 1) {
		label_imu_fix_on_off_ -> setMovie(movie_imu_fix_off_);
		movie_imu_fix_off_ -> start();
	} else {
		label_imu_fix_on_off_ -> setMovie(movie_imu_fix_on_);
		movie_imu_fix_on_ -> start();
	}
}
void WaypointsPathPlanner::timer_for_lidar_status_callback() {
	last_time_in_timer_lidar_status_ = time_from_lidar_status_;
	duration_between_last_time_and_time_from_lidar_status_ = ros::Time::now();
	ros::Duration final_time_for_lidar_status = duration_between_last_time_and_time_from_lidar_status_ - last_time_in_timer_lidar_status_;
	if (final_time_for_lidar_status.sec > 1) {
		label_lidar_status_ -> setPixmap(lidar_status_off_);
		label_lidar_status_ -> show();
	} else {
		label_lidar_status_ -> setPixmap(lidar_status_on_);
		label_lidar_status_ -> show();
	}
}
void WaypointsPathPlanner::timer_for_dms_handle_callback() {
	last_time_in_timer_dms_status_ = time_from_dms_status_;
	duration_between_last_time_and_time_from_dms_status_ = ros::Time::now();
	ros::Duration final_time_for_dms_status_ = duration_between_last_time_and_time_from_dms_status_ - last_time_in_timer_dms_status_;
	if (final_time_for_dms_status_.sec > 1) {
		label_dms_on_off_ -> setMovie(movie_dms_on_off_unknown_);
		movie_dms_on_off_unknown_ -> start();
		timer_for_dms_handle_status_.stop();
	}
}
void WaypointsPathPlanner::timer_for_pod_to_acs_core_callback() {
	last_time_in_timer_pod_to_acs_core_ = time_from_the_pod_to_acs_core_;
	duration_between_last_time_and_time_from_the_pod_to_acs_core_ = ros::Time::now();
	ros::Duration final_time_for_pod_to_acs_core_ = duration_between_last_time_and_time_from_the_pod_to_acs_core_ - last_time_in_timer_pod_to_acs_core_;
	last_time_in_timer_pod_to_acs_aux_ = time_from_the_pod_to_acs_aux_;
	duration_between_last_time_and_time_from_the_pod_to_acs_aux_ = ros::Time::now();
	ros::Duration final_time_for_pod_to_acs_aux_ = duration_between_last_time_and_time_from_the_pod_to_acs_aux_ - last_time_in_timer_pod_to_acs_aux_;
	if ((final_time_for_pod_to_acs_core_.sec > 2) || (final_time_for_pod_to_acs_aux_.sec > 2)) {
		label_battery_default_ -> setMovie(movie_battery_unknown_);
		movie_battery_unknown_ -> start();
		label_autonomy_enable_disable_ -> setMovie(movie_autonomy_enable_on_off_unknown_);
		movie_autonomy_enable_on_off_unknown_ -> start();
		timer_for_battery_status_.stop();
	}
}
void WaypointsPathPlanner::timer_for_bbr_callback() {
	last_time_in_timer_bbr_status_ = time_from_bbr_status_;
	duration_between_last_time_and_time_from_bbr_status_ = ros::Time::now();
	ros::Duration final_time_for_bbr_status = duration_between_last_time_and_time_from_bbr_status_ - last_time_in_timer_bbr_status_;
	if (final_time_for_bbr_status.sec > 1) {
		label_bbr_status_ -> setPixmap(bbr_unknown_);
		label_bbr_status_ -> show();
		timer_for_bbr_status_.stop();
	}
}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(waypoints_path_planner::WaypointsPathPlanner,rviz::Panel)
