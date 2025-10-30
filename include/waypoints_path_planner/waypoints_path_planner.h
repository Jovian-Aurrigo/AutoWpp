/*
 * waypoints_path_planner.h
 *
 *  Created on: 29 Feb 2024
 *      Author: maria-venus
 */

#ifndef WAYPOINTS_PATH_PLANNER_INCLUDE_WAYPOINTS_PATH_PLANNER_H_
#define WAYPOINTS_PATH_PLANNER_INCLUDE_WAYPOINTS_PATH_PLANNER_H_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <queue>
#include <waypoints_path_planner/waypoints_path_planner.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Scalar.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <unordered_map>
#include <map>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <signal.h>
#include <iostream>
#include <thread>
#include <condition_variable>
#include <string.h>
#include <math.h>
#include <tgmath.h>
#include <algorithm>
#include <iterator>
#include <limits>
#include <exception>
#include <istream>
#include <sstream>
#include <memory>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include "boost/shared_ptr.hpp"
#include "boost/filesystem.hpp"
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <QApplication>
#include <QProcess>
#include "qpushbutton.h"
#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <qboxlayout.h>
#include <QLabel>
#include <QTimer>
#include <qradiobutton.h>
#include <qbuttongroup.h>
#include <qcheckbox.h>
#include <QMessageBox>
#include <QTableWidget>
//#include <QtConcurrentRun>
#include <QFuture>
#include <QCursor>
#include <qfiledialog.h>
#include <qinputdialog.h>
#include <qdialog.h>
#include <qdialogbuttonbox.h>
#include <qtooltip.h>
#include <qwidget.h>
#include <qgroupbox.h>
#include <qcombobox.h>
#include <qtextedit.h>
#include <qspinbox.h>
#include <qdial.h>
#include <qmovie.h>
#include <QDate>
#include <qabstractbutton.h>
#include <QGroupBox>
#include <rviz/display.h>
#include <rviz/panel.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/TorusArray.h>
#include <jsk_recognition_msgs/Torus.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#ifdef VEH_TYPE_ACA1
 #include <aca1_message_definitions/av_to_ads_controller.h>
 	using VehicleAVToADSControllerMsg = aca1_message_definitions::av_to_ads_controller;
 #include <aca1_message_definitions/av_to_ads_core.h>
	using VehicleAVToADSCOREMsg = aca1_message_definitions::av_to_ads_core;
 #include <aca1_message_definitions/av_to_ads_aux.h>
	using VehicleAVToADSAUXMsg = aca1_message_definitions::av_to_ads_aux;
 #include <aca1_message_definitions/dms_status.h>
 	using VehicleDMSStatusMsg = aca1_message_definitions::dms_status;
 #include <aca1_message_definitions/blackbox_recorder.h>
 	using VehicleBlackboxRecorderMsg = aca1_message_definitions::blackbox_recorder;
 #include <aca1_message_definitions/aurrigo_timemachine.h>
	using VehicleTimeMachineMsg = aca1_message_definitions::aurrigo_timemachine;
 #include <aca1_message_definitions/aurrigo_system_monitor.h>
	using VehicleSystemMonitorMsg = aca1_message_definitions::aurrigo_system_monitor;
  static std::string topic_prefix = "/aca1/";
 #elif defined(VEH_TYPE_POD)
 #include <podzero_message_definitions/av_to_ads_controller.h>
 	using VehicleAVToADSControllerMsg = podzero_message_definitions::av_to_ads_controller;
 #include <podzero_message_definitions/pod_to_acs_core.h>
	using VehicleAVToADSCOREMsg = podzero_message_definitions::pod_to_acs_core;
 #include <podzero_message_definitions/pod_to_acs_aux.h>
	using VehicleAVToADSAUXMsg = podzero_message_definitions::pod_to_acs_aux;
 #include <podzero_message_definitions/dms_status.h>
 	using VehicleDMSStatusMsg = podzero_message_definitions::dms_status;
 #include <podzero_message_definitions/blackbox_recorder.h>
 	using VehicleBlackboxRecorderMsg = podzero_message_definitions::blackbox_recorder;
 #include <podzero_message_definitions/aurrigo_timemachine.h>
	using VehicleTimeMachineMsg = podzero_message_definitions::aurrigo_timemachine;
 #include <podzero_message_definitions/aurrigo_system_monitor.h>
	using VehicleSystemMonitorMsg = podzero_message_definitions::aurrigo_system_monitor;
 static std::string topic_prefix = "/pod/";
 #elif defined(VEH_TYPE_STL2)
  #include <stl2_message_definitions/av_to_ads_core.h>
	using VehicleAVToADSCOREMsg = stl2_message_definitions::av_to_ads_core;
  #include <stl2_message_definitions/av_to_ads_aux.h>
	using VehicleAVToADSAUXMsg = stl2_message_definitions::av_to_ads_aux;
  #include <stl2_message_definitions/dms_status.h>
 	using VehicleDMSStatusMsg = stl2_message_definitions::dms_status;
  #include <stl2_message_definitions/blackbox_recorder.h>
 	using VehicleBlackboxRecorderMsg = stl2_message_definitions::blackbox_recorder;
  #include <stl2_message_definitions/aurrigo_timemachine.h>
	using VehicleTimeMachineMsg = stl2_message_definitions::aurrigo_timemachine;
  #include <stl2_message_definitions/aurrigo_system_monitor.h>
	using VehicleSystemMonitorMsg = stl2_message_definitions::aurrigo_system_monitor;
 static std::string topic_prefix = "/stl2/";
 #else
  #include <adt3_message_definitions/av_to_ads_controller.h>
 	using VehicleAVToADSControllerMsg = adt3_message_definitions::av_to_ads_controller;
  #include <adt3_message_definitions/av_to_ads_core.h>
	using VehicleAVToADSCOREMsg = adt3_message_definitions::av_to_ads_core;
  #include <adt3_message_definitions/av_to_ads_aux.h>
	using VehicleAVToADSAUXMsg = adt3_message_definitions::av_to_ads_aux;
  #include <adt3_message_definitions/dms_status.h>
 	using VehicleDMSStatusMsg = adt3_message_definitions::dms_status;
  #include <adt3_message_definitions/blackbox_recorder.h>
 	using VehicleBlackboxRecorderMsg = adt3_message_definitions::blackbox_recorder;
  #include <adt3_message_definitions/aurrigo_timemachine.h>
	using VehicleTimeMachineMsg = adt3_message_definitions::aurrigo_timemachine;
  #include <adt3_message_definitions/aurrigo_system_monitor.h>
	using VehicleSystemMonitorMsg = adt3_message_definitions::aurrigo_system_monitor;
static std::string topic_prefix = "/adt3/";  
 #endif
 
#include <waypoints_path_planner/startNavigation.h>
#include <waypoints_path_planner/SectionTrajectory.h>
#include<waypoints_path_planner/SectionTrajectoryArray.h>
#include <waypoints_path_planner/waypointinfo.h>
#include <waypoints_path_planner/podbaselinepointsarray.h>
#include <waypoints_path_planner/waypointinfoarray.h>
#include <waypoints_path_planner/currentpodposewithtrajectoryln.h>
//#include <navigator_interactive_waypoint.h>
#include <waypoints_path_planner/routeinfo.h>
#include <waypoints_path_planner/routeinfoarray.h>
#include <waypoints_path_planner/nogozoneinfo.h>
#include <waypoints_path_planner/nogozoneinfoarray.h>
#include <waypoints_path_planning_tool/waypoints_path_planning_file_management.h>
#include <waypoints_path_planning_tool/waypoints_path_planning_localization_management.h>
#include<waypoints_path_planner/waypoints_path_planner_panel_management.h>
#include <waypoints_path_planner/waypointsfilepath.h>
#include <waypoints_path_planner/status_monitoring_panel.h>

//#include "car_like_planner_ros/PlannerSettings.h"
//#include "car_like_planner_ros/carLocalPlanner.h"
//#include "car_like_planner_ros/visualization.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <waypoints_path_planning_tool/waypoints_path_planning_waypoints_and_no_go_zones_managment.h>

#define foreach BOOST_FOREACH
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace std;
using namespace boost::filesystem;

namespace fs = boost::filesystem;

// ADS Requesting States for reporting

#define ADS_NAVIGATION_STATUS_REQUEST "RN??"
#define ADS_LOADED_ROUTE_STATUS_REQUEST "RR??"
#define ADS_LOADED_SEGMENT_STATUS_REQUEST "RS??"
#define ADS_LOADED_MAP_STATUS_REQUEST "RM??"
#define ADS_CURRENT_WP_INDEX_STATUS_REQUEST "RW??"
#define ADS_TRAJECTORY_PROGRESS_STATUS_REQUEST "RT??"
#define ADS_LOAD_ROUTE_COMMAND "LR"
#define ADS_LOAD_SEGMENT_COMMAND "LS"

// ADS Response States for reporting

/*
#define ADS_NAVIGATION_RUNNING_STATUS_RESPONSE "rn1-"
#define ADS_NAVIGATION_NOT_RUNNING_STATUS_RESPONSE "rn0-"
#define ADS_LOADED_ROUTE_STATUS_RESPONSE rrnn
#define ADS_NO_LOADED_ROUTE_STATUS_RESPONSE rr!!
//#define ADS_LOADED_SEGMENT_STATUS_RESPONSE rsnn
#define ADS_NO_LOADED_SEGMENT_STATUS_RESPONSE rs!!
// #define ADS_LOADED_MAP_STATUS_RESPONSE rmnn
#define ADS_NO_LOADED_MAP_STATUS_RESPONSE rm!!
#define ADS_NO_VALID_CURRENT_WP_STATUS_RESPONSE rw!!
//#define ADS_VALID_CURRENT_WP_STATUS_RESPONSE rwnn
#define ADS_TRAJECTORY_PROGRESS_STATUS_RESPONSE RT??
 */

//ADS Commands

#define ADS_CANCEL_NAVIGATION "BC--"
#define ADS_START_NAVIGATION  "BS--"


namespace waypoints_path_planner
{
class WaypointsPathPlanner: public rviz::Panel
{
	Q_OBJECT

public:
	WaypointsPathPlanner(QWidget * parent = 0);
	~WaypointsPathPlanner();

	struct getlistofwaypoints
	{
		std::vector < waypoints_path_planning_tool::SectionTrajectory > Section;
		bool doeswaypointsexist;

	}s;

	// Custom hash function for unordered_map
	struct pair_hash {
		std::size_t operator()(const std::pair<std::string, std::string>& p) const {
			auto h1 = std::hash<std::string>{}(p.first);
			auto h2 = std::hash<std::string>{}(p.second);
			return h1 ^ (h2 << 1);  // Combine hash values
		}
	};




	virtual void load( const rviz::Config& config );
	virtual void save( rviz::Config config ) const;
	void load_route();
	bool get_routes_from_file();
	void save_route();
	void process_route_request(int route_number);
	void update_waypoint_dynamically(int waypoint_index, double new_waypoint_pos_x, double new_waypoint_pos_y);
	std::string int_to_hexstring(int num);
	std::string get_substring_after_last_underscore(const std::string& str);
	QStringList generate_node_names();
	void initialize_qtable_widget();
	std::map<std::pair<std::string, std::string>, bool> read_matrix_from_yaml(const string& filename);
	std::map<std::pair<std::string, std::string>, bool> read_data_from_qtable_to_unordered_map();
	bool does_route_exist_between_origin_destination(std::string origin, std::string destination);
	std::vector<std::string> segment_path_string(const std::string& input);
	QString generateColumnName(int index);
	QString generateRowName(int index);
	void command_load_segments(std::queue<std::string> command_based_filenames_to_load);
	void set_button_state(QPushButton* button, bool enabled_or_not);


	/*Subscriber's Callback Declarations*/
	void plot_waypoints_2d_nav_goal(geometry_msgs::PoseStamped msg);
	void plot_no_go_zone_callback(const geometry_msgs::PointStamped & msg);
	void dynamic_alignment_callback(waypoints_path_planner::waypointinfo msg);
	void current_vehiclepose_from_localplanner_callback(waypoints_path_planner::currentpodposewithtrajectoryln msg);
	void detailed_route_from_globalplanner_callback(const std_msgs::String::ConstPtr & msg);
	void is_local_goal_reached_callback(std_msgs::Bool msg);
	void pause_set_callback(std_msgs::Bool msg);
#ifdef VEH_TYPE_POD
	void pause_release_from_controller_callback(VehicleAVToADSControllerMsg msg);
#endif
	void localization_pose_callback(geometry_msgs::PoseWithCovarianceStamped msg);
	void av_stop_request_callback(std_msgs::Bool msg);
	void gps_pod_pose_callback(nav_msgs::Odometry msg);
	void ipe_to_ads_callback(std_msgs::String msg);
	void current_waypoint_index_callback(std_msgs::Int32 msg);
	void global_map_path_callback(std_msgs::String msg);
	void end_of_route_subscriber_callback(std_msgs::Bool msg);
	void local_goal_reached_subscriber_callback(std_msgs::Bool msg);
	void lane_switch_callback(std_msgs::Bool msg);
	void waypoint_info_callback(waypoints_path_planner::waypointinfoarray msg);
	void minder_state_callback(std_msgs::String msg);

	/*Function calls to waypoints_path_planning_tool_file_management library*/

	void setFilename(std::string filename);
	void set_waypoints_filename_for_left_and_right_lane(std::string, std::string);
	getlistofwaypoints get_list_of_waypoints_from_file(std::string filename, std::vector < waypoints_path_planner::SectionTrajectory > ListPoses);
	void set_route_filename(std::string route_filename);
	void publish_waypoint_paths();


	/*Function calls to waypoints_path_planning_tool_no_go_zone_management library*/

	void publish_empty_no_go_zone();
	void clear_waypoints_descriptors_arrays(); //name change
	void clear_no_go_zone_objects_before_loading_file(); //name change


	/*Function calls to waypoints_path_planning_tool_localization_management library*/
	void publish_pose_estimate_from_waypoints_file(std::string filename);
	bool start_av_navigation_process(std::vector < waypoints_path_planning_tool::SectionTrajectory > route);
	bool cancel_av_navigation_process(bool navigation_started);
	double calculate_distance_between_two_points(geometry_msgs::Pose point1, geometry_msgs::Pose point2);

	/*Function calls to waypoints_path_planner_panel_management library*/
	void rviz_panel_locked();
	void rviz_panel_unlocked();
	void load_route_process(std::string main_lane_filename, std::string left_lane_filename, std::string right_lane_filename);

private:

	/*Waypoint Path Planner Panel Object's Declaration*/

	QGridLayout * gridLayout_forwaypoint_panel_;

	/* New Separated Components */
	std::unique_ptr<StatusMonitoringPanel> status_panel_;

	QPushButton * load_waypoints_file_;
	QPushButton * compute_segment_;
	QPushButton * start_navigation_;
	QPushButton * cancel_navigation_;
	QPushButton * clear_waypoint_markers_;
	QPushButton * create_waypoint_file_;
	QPushButton * place_lidar_waypoints_;
	QPushButton * place_gps_waypoints_;
	QPushButton * pause_release_;
	QPushButton * vehicle_pose_estimate_;
	QPushButton * delete_no_go_zone_markers_in_file_and_rviz_;
	QPushButton * change_map_;
	QPushButton * delete_for_waypoints_;
	QPushButton * toggle_lock_unlock_;
	QPushButton * left_lane_switch_;
	QPushButton * right_lane_switch_;
	QPushButton * create_route_file_;
	QPushButton * load_route_file_;
	QPushButton * set_route_file_;
	QPushButton * run_route_1_file_;
	QPushButton * run_route_2_file_;
	QPushButton * run_route_3_file_;
	QPushButton * run_route_4_file_;

	QByteArray string_, string1_;
	QString str_, str1_, str2_, str4_;
	QProcess qprocess_audio;

	/* Debug Messages*/

	QTextEdit * debug_message_edit_;

	//QRadioButton * av_drive_mode_;
	//QRadioButton * av_drive_mode_crab_;

	/* ADS to IPE */
	float adt_x;
    float adt_y;
	geometry_msgs::Pose end_point_;

	/*Routes Panel*/

	QWidget * routes_manipulation_panel_;
	QGridLayout * grid_layout_for_routes_manipulation_panel_;
	QLabel * routes_index_label_;
	QLabel * routes_filename_label_;
	QSpinBox * index_spinbox_;
	QPushButton * add_route_button_;
	QPushButton * delete_route_button_;
	QPushButton * save_route_button_;
	QLabel * segment_filename_label_;
	QLineEdit * segment_filename_lineedit_;
	QLineEdit * routes_filename_lineedit_;
	QTableWidget *table_widget_;
	QLabel *route_possibility_;
	QLabel *origin_input_;
	QLabel *destination_input_;
	QLineEdit *input_origin_;
	QLineEdit *input_destination_;
	QPushButton *check_route_possibility_;

	ros::NodeHandle nh_;
	std::string WorkingFolder_;
	bool does_main_lane_working_folder_exist_, does_right_lane_working_folder_exist_, does_left_lane_working_folder_exist_, does_temp_lane_working_folder_exist_;
	bool ifWorkingFileExist_, if_route_file_exist_;
	bool is_clicked;
	std_msgs::String safety_supervisor_msg;
	std::string filename_, routes_filename_, left_lane_filename_, right_lane_filename_;
	std::string toggle_button_lock_file_, toggle_button_unlock_file_;

	std::string quit_process_file_, command;

	QString qmap_file_path_;
	std::string map_file_path_;

	/* Data required for CarPlanner */

	carlike_local_planner::CarlikeConfig cfg_;
	carlike_local_planner::CarLikeVisualizationPtr Visualizer_;
	boost::shared_ptr < CarLocalPlanner > Planner_;

	geometry_msgs::Pose current_vehicle_pose_from_local_planner_;
	int line_number_of_vehicle_pose_in_the_global_plan_;
	waypoints_path_planner::currentpodposewithtrajectoryln current_pose_and_ln_number_from_local_planner_;
	bool is_filepath_received;
	std::string file_location_of_vehicle_global_plan;
	std_msgs::Bool pause_set_;
	std_msgs::Bool is_local_goal_reached_;
	bool is_goal_reached_;
	std_msgs::Bool is_rviz_locked_unlocked_;
	int xbox_buttons_;
	int pod_state_;
	int old_button_state_;
	std_msgs::Bool is_av_navigating_msg_;
	bool does_waypoints_exist_in_main_lane_, does_waypoints_exist_in_left_lane_, does_waypoints_exist_in_right_lane_;
	bool navigation_started_, new_file_just_loaded_;
	bool check_localization_pose_, check_gps_pod_pose_;
	visualization_msgs::InteractiveMarker vehicle_pose_, new_pose_, old_pose_;
	visualization_msgs::InteractiveMarker vehicle_pose_from_gps_;
	double distance_from_home_, radius_of_the_no_go_zone_;
	geometry_msgs::Pose home_pose_;
	std_msgs::String map_file_path_msg;
	uint8_t pod_state_msg_;
	int counter_for_2d_nav_goal_;
	int index_value_changed_, index_value_changed_counter_;
	int waypoint_position_, waypoints_size_, position_, no_go_zone_position_;
	std::vector < std::string > routes_file_array_;
	int current_waypoint_index_;
	std::string route_filename_for_ipe_server_, segment_filename_for_ipe_server_, previous_destination_string_, previous_origin_string_;
	std::vector<std::string> nodes;
	std::vector<std::string> segments;
	std::map<std::pair<std::string, std::string>, bool> matrix_map;
	bool does_route_exist_;
	waypoints_path_planner::routeinfo route_info_;
	waypoints_path_planner::routeinfoarray route_info_array_;
	std::queue<std::string> command_based_filenames_to_load;
	bool filename_set_, left_right_filename_set_;
	bool end_of_route_, local_goal_reached_;
	bool boot_safe;
	waypoints_path_planner::waypointsfilepath waypoints_file_path_;
	std::string last_minder_state_;



	/*Subscribers*/
	ros::Subscriber gps_pose_subscriber_;
	ros::Subscriber global_plan_subscriber_;
	ros::Subscriber global_map_path_subscriber_;
	ros::Subscriber current_vehicle_position_subscriber_;
	ros::Subscriber is_local_goal_reached_subscriber_;
	ros::Subscriber pause_set_subscriber_;
	ros::Subscriber pause_release_subscriber_through_xbox_controller_;
	ros::Subscriber localization_pose_subscriber_;
	ros::Subscriber stop_requested_subscriber_;
	ros::Subscriber dynamic_alignment_subscriber_;
	ros::Subscriber point_clicked_from_rviz_to_plot_no_go_zones_subscriber_;
	ros::Subscriber nav_goal_to_plot_waypoints_subscriber_;
	ros::Subscriber ipe_to_ads_subscriber_;
	ros::Subscriber current_waypoint_index_subscriber_;
	ros::Subscriber end_of_route_subscriber_;
	ros::Subscriber local_goal_reached_subscriber_;
	ros::Subscriber lane_switch_subscriber_;
	ros::Subscriber waypointinfo_subscriber_;
	ros::Subscriber minder_state_subscriber_;


	/*Publishers*/
	ros::Publisher safety_supervisor_name_publisher_;
	ros::Publisher lock_unlock_rviz_status_publisher_;
	ros::Publisher pause_release_publisher_;
	ros::Publisher map_file_path_publisher_;
	ros::Publisher ads_to_ipe_publisher_;
	ros::Publisher file_path_publisher_;

	ros::ServiceClient start_navigation_service_, cancel_navigation_service_;

	/*Waypoint Path Planner's Q-SLOTS Function Declarations*/

	public Q_SLOTS:

	/* Main Panel */

	void set_poseestimate_qslot();
	void load_newmap_qslot();
	void start_av_navigation_qslot(void);
	void cancel_navigation_qslot(void);
	void lock_unlock_rvizedit_qslot();
	void switch_to_leftlane_qslot();
	void switch_to_rightlane_qslot();
	void create_waypoint_file_qslot();
	void create_route_file_qslot();
	void load_waypoints_file_qslot();
	void load_route_file_qslot();
	void clearall_waypoint_markers_qslot();
	void place_lidar_waypoints_qslot();
	void place_gps_waypoints_qslot();
	void compute_segment_qslot();
	void pause_release_qslot();
	void delete_no_go_zone_markers_in_file_and_rviz_qslot();
	void set_route_file_qslot();
	void run_route_1_qslot();
	void run_route_2_qslot();
	void run_route_3_qslot();
	void run_route_4_qslot();
	void index_value_changed_qslot(int i);
	void add_route_qslot();
	void delete_segment_qslot();
	void save_route_qslot();
	void reset_matrix_qslot();
	void cell_clicked_qslot(int x, int y);
	void does_route_exist_qslot();


};

}






#endif /* WAYPOINTS_PATH_PLANNER_INCLUDE_WAYPOINTS_PATH_PLANNER_H_ */
