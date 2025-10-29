/*
 * navigator_interactive_waypoint.h
 *
 *  Created on: 20 Aug 2019
 *      Author: maria
 */

#ifndef INCLUDE_NAVIGATOR_INTERACTIVE_WAYPOINT_H_
#define INCLUDE_NAVIGATOR_INTERACTIVE_WAYPOINT_H_

#include <ros/ros.h>

#include "geometry_msgs/PoseArray.h"

#include "geometry_msgs/PoseStamped.h"

#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>

#include <tf/tf.h>

#include <tf/transform_listener.h>

#include <ecl/geometry/angle.hpp>

#include "waypoints_path_planner.h"

#include "waypoints_path_planner/startNavigation.h"

#include "waypoints_path_planner/SectionTrajectory.h"

#include "waypoints_path_planning_tool/SectionTrajectory.h"

#include<waypoints_path_planner/SectionTrajectoryArray.h>

#include "waypoints_path_planner/podbaselinepointsarray.h"

#include "car_like_planner_ros/PlannerSettings.h"

#include <std_srvs/Trigger.h>

#include <std_msgs/String.h>

#include <std_msgs/Int32.h>

#include <std_msgs/Bool.h>

#include <std_msgs/Float32.h>

#include <QProcess>

#include <queue>

#ifdef VEH_TYPE_ACA1
 #include <aca1_message_definitions/ads_to_av.h>
	using VehicleADSToAVMsg = aca1_message_definitions::ads_to_av;
  #include <aca1_message_definitions/av_to_ads_core.h>
	using VehicleAVToADSCOREMsg = aca1_message_definitions::av_to_ads_core;
  #include <aca1_message_definitions/av_to_ads_aux.h>
	using VehicleAVToADSAUXMsg = aca1_message_definitions::av_to_ads_aux;
  #include <aca1_message_definitions/waypointnav_reqs_ads_to_av.h>
std::string topic_prefix_ = "/aca1/";
       using VehicleAVWaypointReqADSToAV = aca1_message_definitions::waypointnav_reqs_ads_to_av;
#elif defined(VEH_TYPE_POD)
  #include <podzero_message_definitions/acs_to_pod.h>
	using VehicleADSToAVMsg = podzero_message_definitions::ads_to_av;
  #include <podzero_message_definitions/pod_to_acs_core.h>
	using VehicleAVToADSCOREMsg = podzero_message_definitions::av_to_ads_core;
  #include <podzero_message_definitions/pod_to_acs_aux.h>
	using VehicleAVToADSAUXMsg = podzero_message_definitions::av_to_ads_aux;
  #include <podzero_message_definitions/waypointnav_reqs_ads_to_av.h>
       using VehicleAVWaypointReqADSToAV = podzero_message_definitions::waypointnav_reqs_ads_to_av;
  std::string topic_prefix_ = "/pod/";
#elif defined(VEH_TYPE_STL2)
  #include <stl2_message_definitions/ads_to_av.h>
	using VehicleADSToAVMsg = stl2_message_definitions::ads_to_av;
  #include <stl2_message_definitions/av_to_ads_core.h>
	using VehicleAVToADSCOREMsg = stl2_message_definitions::av_to_ads_core;
  #include <stl2_message_definitions/av_to_ads_aux.h>
	using VehicleAVToADSAUXMsg = stl2_message_definitions::av_to_ads_aux;
  #include <stl2_message_definitions/waypointnav_reqs_ads_to_av.h>
        using VehicleAVWaypointReqADSToAV = stl2_message_definitions::waypointnav_reqs_ads_to_av;
std::string topic_prefix_ = "/stl2/";
#else
  #include <adt3_message_definitions/ads_to_av.h>
	using VehicleADSToAVMsg = adt3_message_definitions::ads_to_av;
  #include <adt3_message_definitions/av_to_ads_core.h>
	using VehicleAVToADSCOREMsg = adt3_message_definitions::av_to_ads_core;
  #include <adt3_message_definitions/av_to_ads_aux.h>
	using VehicleAVToADSAUXMsg = adt3_message_definitions::av_to_ads_aux;
  #include <adt3_message_definitions/waypointnav_reqs_ads_to_av.h>
       using VehicleAVWaypointReqADSToAV = adt3_message_definitions::waypointnav_reqs_ads_to_av;
std::string topic_prefix_ = "/adt3/";  
#endif

class navigator_interative_waypoints {
  public: navigator_interative_waypoints(void);
  ~navigator_interative_waypoints(void) {}

  void run(void);

  private: void pause_subscriberCallback(std_msgs::Int32 msg);
  std_msgs::Int32 pause_button_release_;
  void ontimerforlaseroveride(void);
  void ontimerforindicators(void);
  void waypointinfoCallback(waypoints_path_planner::waypointinfoarray);
  void lastwaypointSubscriberCallback(geometry_msgs::Pose msg);
  void podtoacsCoreCallback(VehicleAVToADSCOREMsg msg);
  void podtoacsAuxCallback(VehicleAVToADSAUXMsg msg);
  void sdsGoalAchievedallback(std_msgs::Float32 msg);
  void wpp_to_navigator_route_list_callback(waypoints_path_planner::SectionTrajectoryArray msg);
  void virtualPauseCallback(const std_msgs::Bool::ConstPtr& msg);


  protected:

    bool startNavigationCallback(waypoints_path_planner::startNavigationRequest & request,
      waypoints_path_planner::startNavigationResponse & response);
  bool cancelNavigationCallback(std_srvs::TriggerRequest & request, std_srvs::TriggerResponse & response);

  bool sendWaypointPlanner(void);
  bool getRobotPosGL(tf::StampedTransform &robot_gl);
  bool isReadyToMoveNextWaypoint(void);
  void sleep(void);

  int getClosestWaypointToCurrentPosition(void);
  void sendGoal(geometry_msgs::Pose Pose);
  void ipe_command_route_start_navigation();

  double ErrorPosDetectingInitPoint_;
  double ErrorHeadingDetectingInitPoint_;

  //sds nav mode
  double sds_nav_mode;
  void sds_nav_mode_Callback(std_msgs::Float32 msg);

  double DistanceFromGoalToMoveNextWaypoint_;
  double HeadingFromGoalToMoveNextWaypoint_;
  double min_value_ride_height_front_;
  double min_value_ride_height_rear_;

  actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction > move_base_action_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Rate rate_;
  std::string robot_frame_,
  world_frame_;

  ros::ServiceServer start_navigation_,
  cancel_navigation_;
  ros::ServiceClient planner_settings_srv_;

  std::vector < waypoints_path_planner::SectionTrajectory > ListWaypoints_;
  std::vector < waypoints_path_planner::SectionTrajectory > ::iterator CurrentWaypoint_;
  std_msgs::Bool isNavigating_;

  std::string CarlikePlannerSettings_;

  // Publish pause release trigger to HMI

  ros::Publisher hmi_pause_indicator_;
  std_msgs::String pause_message_;

  // Subscribe to pause release from button press

  ros::Subscriber pause_subscriber_;
  QProcess qprocess_audio;

  // Publisher for Laser Zones, Horn, Left and Right Indicator

  ros::Publisher interactive_information_publisher_;
  ros::Publisher acs_pod_information_publisher_;
  ros::Publisher is_pod_navigating_publisher_;
  
  VehicleADSToAVMsg acs_pod_decriptor_;
  VehicleAVWaypointReqADSToAV interactive_information_descriptor_;
  
  // Timer for Laser override

  ros::Timer timer_publisher_for_laser_overide;
  double rateoftimeforlaseroveride_;

  // Timer for Indicator override

  ros::Timer timer_publisher_for_indicator_overide;
  double rateoftimeforindicator_;

  // Waypoint info subscriber

  ros::Subscriber waypointinfo_subscriber_;
  waypoints_path_planner::waypointinfo waypoint_info_;
  waypoints_path_planner::waypointinfoarray waypoint_info_array_;

  geometry_msgs::Pose start_point_,
  end_point_;
  // Draw a line through the pod base parameters

  int length_of_line_through_the_pod_;
  double angle_of_pod;
  double pod_angle_in_radians;
  geometry_msgs::Point point2_,
  point1_;
  double slope_,
  intercept_;
  int dx,
  dy,
  dxc,
  dx1,
  dyc,
  dy1,
  cross;
  bool waypoint_goal_crossed_manually_;
  ros::Publisher line_visualization_points_publisher_;
  waypoints_path_planner::podbaselinepointsarray points_for_Waypoint_Skipping_rviz_visualization_;
  ros::Subscriber pod_to_acs_core_sub_;
  ros::Subscriber sds_goal_achieved_sub_;
  ros::Subscriber virtual_pause_sub_;
  int msg_For_manual_state_;

  ros::Subscriber pod_to_acs_aux_sub_;
  int msg_For_ride_height_front_;
  int msg_For_ride_height_rear_;
  int msg_For_sds_goal_achieved_;

  //pause status for icon
  ros::Publisher pause_set_publisher_;
  std_msgs::Bool pause_set_;

  // pause time publisher

  ros::Publisher current_pause_time_publisher_;
  std_msgs::Int32 current_pause_time_;

  // AV Drive mode publisher

  ros::Publisher current_drive_mode_publisher_;
  std_msgs::Bool current_drive_mode_;
  ros::Publisher next_drive_mode_publisher_;
  std_msgs::Bool next_drive_mode_;

  // AV Crab Height Publisher

  ros::Publisher crab_height_publisher_;
  std_msgs::Float32 crab_height_;
  

  // Debug string Publisher
  ros::Publisher debug_string_publisher_;
  std_msgs::String debug_string_msg_;
  ros::Publisher debug_string_2_publisher_;
  std_msgs::String debug_string_2_msg_;
  ros::Publisher debug_string_3_publisher_;
  std_msgs::String debug_string_3_msg_;

  // Current Waypoint publisher

  ros::Publisher current_waypoint_publisher_;
  geometry_msgs::Pose current_waypoint_pose_;

  // Next Waypoint Publisher

  ros::Publisher next_waypoint_publisher_;
  geometry_msgs::Pose next_waypoint_pose_;

  // Waypoint index publisher

  ros::Publisher current_waypoint_index_publisher_;
  std_msgs::Int32 current_waypoint_index_;

  ros::Publisher current_waypoint_velocity_publisher_;
  std_msgs::Float32 current_waypoint_velocity_;

  //sds_nav_mode
  ros::Subscriber sds_act_nav_mode_float32_sub_;

  //autoconnect

  ros::Publisher end_of_route_current_route_publisher_;
  bool end_of_route_;
  std_msgs::Bool end_of_route;

  //dynamic obstacle avoidance
  bool virtual_pause_;

};

#endif /* INCLUDE_NAVIGATOR_INTERACTIVE_WAYPOINT_H_ */
