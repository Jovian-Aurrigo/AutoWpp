/*
 * navigator_interactive_waypoint.cpp
 *
 *  Created on: 20 Aug 2019
 *      Author: maria
 */

#include "waypoints_path_planner/navigator_interactive_waypoint.h"

navigator_interative_waypoints::navigator_interative_waypoints(void)
: move_base_action_("move_base", true),
  ErrorPosDetectingInitPoint_(2),  // 2
  ErrorHeadingDetectingInitPoint_(0.35),
  DistanceFromGoalToMoveNextWaypoint_(1.0),  // 1.0
  HeadingFromGoalToMoveNextWaypoint_(0.4),   // 0.4
  rate_(20),
  CarlikePlannerSettings_("/CarlikeLocalPlannerROS/Settings"),
  rateoftimeforlaseroveride_(0.07),
  rateoftimeforindicator_(0.1),
  waypoint_goal_crossed_manually_(false),
  end_of_route_(false),
  min_value_ride_height_front_(73), //ADT3-05 75,
  min_value_ride_height_rear_(51),  //ADT3-05 55,
  virtual_pause_(false)

{
	while ((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
	{
		ROS_INFO("Waiting...");
	}

	ros::NodeHandle private_nh("~");
	private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
	private_nh.param("world_frame", world_frame_, std::string("map"));
	private_nh.param(
			"DistanceFromGoalToMoveNextWaypoint_",
			DistanceFromGoalToMoveNextWaypoint_,
			DistanceFromGoalToMoveNextWaypoint_);
	private_nh.param(
			"HeadingFromGoalToMoveNextWaypoint_",
			HeadingFromGoalToMoveNextWaypoint_,
			HeadingFromGoalToMoveNextWaypoint_);
	private_nh.param("ErrorPosDetectingInitPoint_", ErrorPosDetectingInitPoint_, ErrorPosDetectingInitPoint_);
	private_nh.param("ErrorHeadingDetectingInitPoint_", ErrorHeadingDetectingInitPoint_, ErrorHeadingDetectingInitPoint_);
	private_nh.param("min_value_ride_height_front", min_value_ride_height_front_, min_value_ride_height_front_);
	private_nh.param("min_value_ride_height_rear", min_value_ride_height_rear_, min_value_ride_height_rear_);

	current_waypoint_index_.data = -1;

	// DistanceFromGoalToMoveNextWaypoint_ = 1.5;
	
	ROS_INFO_STREAM("HeadingFromGoalToMoveNextWaypoint_: " << HeadingFromGoalToMoveNextWaypoint_);
	ROS_INFO_STREAM("DistanceFromGoalToMoveNextWaypoint_: " << DistanceFromGoalToMoveNextWaypoint_);

	double max_update_rate;
	private_nh.param("max_update_rate", max_update_rate, 10.0);
	rate_ = ros::Rate(max_update_rate);

	end_of_route_current_route_publisher_ = nh_.advertise<std_msgs::Bool>("/end_of_route", 1, true);
	start_navigation_ =
			nh_.advertiseService("/start_navigation", &navigator_interative_waypoints::startNavigationCallback, this);
	cancel_navigation_ =
			nh_.advertiseService("/cancel_navigation", &navigator_interative_waypoints::cancelNavigationCallback, this);
	planner_settings_srv_ = nh_.serviceClient<car_like_planner_ros::PlannerSettings>(CarlikePlannerSettings_);
	hmi_pause_indicator_ = nh_.advertise<std_msgs::String>("/hmi_pause_release", 100);
	pause_set_publisher_ = nh_.advertise<std_msgs::Bool>("/pause_set", 100);
	pause_subscriber_ =
			nh_.subscribe("/release_pause", 1, &navigator_interative_waypoints::pause_subscriberCallback, this);
	interactive_information_publisher_ =
			nh_.advertise<VehicleAVWaypointReqADSToAV>("/waypointnav_reqs_acs_to_pod", 1, true);
	acs_pod_information_publisher_ =
			nh_.advertise<VehicleADSToAVMsg>("/acs_to_pod_messages_to_gateway", 1, true);
	waypointinfo_subscriber_ =
			nh_.subscribe("/waypoint_info", 1, &navigator_interative_waypoints::waypointinfoCallback, this);
	is_pod_navigating_publisher_ = nh_.advertise<std_msgs::Bool>("/isPodNavigating", 1, true);
	timer_publisher_for_laser_overide = nh_.createTimer(
			ros::Duration(rateoftimeforlaseroveride_),
			boost::bind(&navigator_interative_waypoints::ontimerforlaseroveride, this));
	timer_publisher_for_indicator_overide = nh_.createTimer(
			ros::Duration(rateoftimeforindicator_),
			boost::bind(&navigator_interative_waypoints::ontimerforindicators, this));
	isNavigating_.data = false;
	is_pod_navigating_publisher_.publish(isNavigating_);
	std::cout << "Navigator On !!!!" << std::endl;
	current_waypoint_velocity_publisher_ = nh_.advertise<std_msgs::Float32>("/current_waypoint_velocity", 1, true);
	current_pause_time_publisher_ = nh_.advertise<std_msgs::Int32>("/current_pause_time", 1, true);
	current_drive_mode_publisher_ = nh_.advertise<std_msgs::Bool>("/current_drive_mode", 1, true);
	next_drive_mode_publisher_ = nh_.advertise<std_msgs::Bool>("/next_drive_mode", 1, true);
	crab_height_publisher_ = nh_.advertise<std_msgs::Float32>("/crab_height", 1, true);
	current_waypoint_index_publisher_ = nh_.advertise<std_msgs::Int32>("/current_waypoint_index", 1, true);
	current_waypoint_publisher_ = nh_.advertise<geometry_msgs::Pose>("/current_waypoint", 1, true);
	next_waypoint_publisher_ = nh_.advertise<geometry_msgs::Pose>("/next_waypoint", 1, true);

	debug_string_publisher_ = nh_.advertise<std_msgs::String>("/wpp_debug_str", 1, true);
	debug_string_2_publisher_ = nh_.advertise<std_msgs::String>("/wpp_debug_str_2", 1, true);
	debug_string_3_publisher_ = nh_.advertise<std_msgs::String>("/wpp_debug_str_3", 1, true);

	line_visualization_points_publisher_ =
			nh_.advertise<waypoints_path_planner::podbaselinepointsarray>("/points_for_pod_base_line_construction", 1, true);
	pod_to_acs_core_sub_ =
			nh_.subscribe(topic_prefix_+"av_to_ads_core", 1, &navigator_interative_waypoints::podtoacsCoreCallback, this);
	pod_to_acs_aux_sub_ = nh_.subscribe(topic_prefix_+"av_to_ads_aux", 1, &navigator_interative_waypoints::podtoacsAuxCallback, this);
	sds_goal_achieved_sub_ =
			nh_.subscribe("/sds_v2/goal_achieved", 1, &navigator_interative_waypoints::sdsGoalAchievedallback, this);
	msg_For_sds_goal_achieved_ = -1;
	// sds sub
	sds_act_nav_mode_float32_sub_ =
			nh_.subscribe("/sds_v2/active_nav_mode", 1, &navigator_interative_waypoints::sds_nav_mode_Callback, this);
	virtual_pause_sub_ = 
			nh_.subscribe("/virtual_pause", 1, &navigator_interative_waypoints::virtualPauseCallback, this);
}

void navigator_interative_waypoints::podtoacsCoreCallback(VehicleAVToADSCOREMsg msg)
{
	msg_For_manual_state_ = msg.AvState;
}

void navigator_interative_waypoints::sdsGoalAchievedallback(std_msgs::Float32 msg)
{
	// if msg.data == -1 then goal not achieved, else we have achieved the goal number sent
	msg_For_sds_goal_achieved_ = (int)msg.data;
}

void navigator_interative_waypoints::podtoacsAuxCallback(VehicleAVToADSAUXMsg msg)
{
#ifdef VEH_TYPE_ADT3
	msg_For_ride_height_front_ = msg.FrontAxleRideHeight;
	msg_For_ride_height_rear_ = msg.RearAxleRideHeight;
#else
	msg_For_ride_height_front_ = 100;
	msg_For_ride_height_rear_ = 100;
#endif
}
void navigator_interative_waypoints::waypointinfoCallback(waypoints_path_planner::waypointinfoarray msg)
{
	waypoint_info_array_ = msg;
	for (int i = 0; i < waypoint_info_array_.waypointinfodata.size(); i++)
	{
		start_point_ = waypoint_info_array_.waypointinfodata.at(0).waypointpose;
		end_point_ =
				waypoint_info_array_.waypointinfodata.at(waypoint_info_array_.waypointinfodata.size() - 1).waypointpose;
	}
}

// sds_nav_mode
void navigator_interative_waypoints::sds_nav_mode_Callback(std_msgs::Float32 msg)
{
#ifdef VEH_TYPE_ADT3
	sds_nav_mode = msg.data;
#else
	sds_nav_mode = 0;
#endif
}

void navigator_interative_waypoints::ontimerforlaseroveride()
{
	interactive_information_publisher_.publish(interactive_information_descriptor_);
}
void navigator_interative_waypoints::ontimerforindicators()
{
	acs_pod_information_publisher_.publish(acs_pod_decriptor_);
}
void navigator_interative_waypoints::pause_subscriberCallback(std_msgs::Int32 msg)
{
	std::cout << "Im in pausesubscriberCallback" << std::endl;
	pause_button_release_ = msg;
	std::cout << pause_button_release_ << std::endl;
}

bool navigator_interative_waypoints::startNavigationCallback(
		waypoints_path_planner::startNavigationRequest& request,
		waypoints_path_planner::startNavigationResponse& response)
{
	ROS_INFO_STREAM("navigator_interative_waypoints::startNavigationCallback");

	end_of_route.data = false;
	// end_of_route_current_route_publisher_.publish(end_of_route);
	ListWaypoints_.clear();
	ListWaypoints_ = request.Route;
	int index = -1;
	index = getClosestWaypointToCurrentPosition();
	CurrentWaypoint_ = std::next(ListWaypoints_.begin(), index);
	current_waypoint_index_.data = -1;
	current_waypoint_index_.data = CurrentWaypoint_->index;
	current_waypoint_index_publisher_.publish(current_waypoint_index_);
	
	// //SPB
	// std::stringstream str_stream;
	// str_stream << "Index value: " << index << " CurrentWaypoint_ " << CurrentWaypoint_->index;
	// debug_string_msg_.data = str_stream.str();
	// debug_string_publisher_.publish(debug_string_msg_);
	
	ROS_INFO_STREAM("Index value= " << index);

	if (!sendWaypointPlanner())
	{
		response.status = false;
		isNavigating_.data = false;
		return false;
	}

	response.status = true;
	isNavigating_.data = true;

	return true;
}

bool navigator_interative_waypoints::cancelNavigationCallback(
		std_srvs::TriggerRequest& request,
		std_srvs::TriggerResponse& response)
{
	ListWaypoints_.clear();
	move_base_action_.cancelAllGoals();

	response.success = true;
	isNavigating_.data = false;
	is_pod_navigating_publisher_.publish(isNavigating_);
	return true;
}

void navigator_interative_waypoints::virtualPauseCallback(
		const std_msgs::Bool::ConstPtr& msg)
{
    virtual_pause_ = msg->data;  // updates shared state
}

bool navigator_interative_waypoints::sendWaypointPlanner(void)
{
	std::string string1("NOFILE");
	std::stringstream str_stream_1;

	if (std::next(CurrentWaypoint_) == ListWaypoints_.end())
	{
		ListWaypoints_.clear();
		move_base_action_.cancelAllGoals();
		isNavigating_.data = false;
		is_pod_navigating_publisher_.publish(isNavigating_);
		return false;
	}

	car_like_planner_ros::PlannerSettingsRequest RequestPlannerSettings;
	car_like_planner_ros::PlannerSettingsResponse ReplyPlannerSettings;

	RequestPlannerSettings.InitVelocity = CurrentWaypoint_->velocity;
	RequestPlannerSettings.EndVelocity = std::next(CurrentWaypoint_)->velocity;
	RequestPlannerSettings.PathFilenamePreComputedIteration = std::next(CurrentWaypoint_)->trajectory_file;
	RequestPlannerSettings.InitPose = CurrentWaypoint_->waypoint_pose;
	RequestPlannerSettings.EndPose = std::next(CurrentWaypoint_)->waypoint_pose;

	if (RequestPlannerSettings.InitVelocity == RequestPlannerSettings.EndVelocity)
		RequestPlannerSettings.ConstantVelocity = true;
	else
		RequestPlannerSettings.ConstantVelocity = false;

	ROS_INFO_STREAM("Request Planner: " << RequestPlannerSettings);

	bool successServiceCall = false;
	const int MaxNumberTries = 5;
	int counterRetries = 0;

	do
	{
		if (!planner_settings_srv_.exists())
			ROS_INFO("Carefully service does not exist we might need to reconnect.");
		planner_settings_srv_.waitForExistence(ros::Duration(1));
		if (!planner_settings_srv_.exists())
			ROS_INFO("It does not exist after waiting for existence!!!. There is a problem.");

		successServiceCall = planner_settings_srv_.call(RequestPlannerSettings, ReplyPlannerSettings);
		ROS_INFO("successServiceCall after call %d.", successServiceCall);
		ROS_INFO("ReplyPlannerSettings.status %d.", ReplyPlannerSettings.status);

		successServiceCall &= ReplyPlannerSettings.status;
		ROS_INFO("successServiceCall after checking status returned %d.", successServiceCall);

		if (ReplyPlannerSettings.status)
		{

			current_waypoint_index_.data = CurrentWaypoint_->index;
			current_waypoint_index_publisher_.publish(current_waypoint_index_);
			current_pause_time_.data = CurrentWaypoint_->pause_time;
			current_pause_time_publisher_.publish(current_pause_time_);


			// current_drive_mode_.data = CurrentWaypoint_->av_drive_mode;
			// //SPB nned to allow drive mode to get us to raise when met wp to raise
			// if (std::prev(CurrentWaypoint_) != ListWaypoints_.begin())
			// {
			// 	if (((CurrentWaypoint_->crab_wheel_height_request == 100) && (CurrentWaypoint_->av_drive_mode == 0)) &&
			// 		 ((std::prev(CurrentWaypoint_)->crab_wheel_height_request == 0) && (std::prev(CurrentWaypoint_)->av_drive_mode == 1))) 
			// 	{
			// 		if ((msg_For_ride_height_front_ < min_value_ride_height_front_) || (msg_For_ride_height_rear_ < min_value_ride_height_rear_))  // SPB not the right height yet
			// 		{
			// 			current_drive_mode_.data = true;
			// 		}
			// 	}
			// }
			// current_drive_mode_publisher_.publish(current_drive_mode_);

			// //SPB
			// str_stream_1 << "current_drive_mode_.data: " << current_drive_mode_.data << " CurrentWaypoint_->av_drive_mode " << CurrentWaypoint_->av_drive_mode;
			// debug_string_msg_.data = str_stream_1.str();
			// debug_string_publisher_.publish(debug_string_msg_);


			if (std::next(CurrentWaypoint_) != ListWaypoints_.end())
			{
				// we are not at the last waypoint so we can publish the next one
				next_drive_mode_.data = std::next(CurrentWaypoint_)->av_drive_mode;
				next_drive_mode_publisher_.publish(next_drive_mode_);
			}
			current_waypoint_pose_ = CurrentWaypoint_->waypoint_pose;
			current_waypoint_publisher_.publish(current_waypoint_pose_);
			next_waypoint_pose_ = std::next(CurrentWaypoint_)->waypoint_pose;
			next_waypoint_publisher_.publish(next_waypoint_pose_);
			// crab_height_.data = CurrentWaypoint_ -> crab_wheel_height_request;
			// crab_height_publisher_.publish(crab_height_);
			current_waypoint_velocity_.data = CurrentWaypoint_->velocity;
			current_waypoint_velocity_publisher_.publish(current_waypoint_velocity_);

			if (CurrentWaypoint_->set_left_indicator == 1)
			{
				std::cout << "Information Left LeftIndicator" << CurrentWaypoint_->set_left_indicator << std::endl;

				acs_pod_decriptor_.DiReqAcs = (uint8_t)2;
				acs_pod_decriptor_.InteriorLightsReqAcs = 10;
				acs_pod_decriptor_.HeadlampReqAcs = 2;
			}
			if (CurrentWaypoint_->set_right_indicator == 1)
			{
				acs_pod_decriptor_.DiReqAcs = (uint8_t)3;
				acs_pod_decriptor_.InteriorLightsReqAcs = 10;
				acs_pod_decriptor_.HeadlampReqAcs = 2;
			}
			if ((CurrentWaypoint_->set_right_indicator == 0) && (CurrentWaypoint_->set_left_indicator == 0))
			{
				acs_pod_decriptor_.DiReqAcs = (uint8_t)0;
				acs_pod_decriptor_.InteriorLightsReqAcs = 10;
				acs_pod_decriptor_.HeadlampReqAcs = 2;
			}
			else if ((CurrentWaypoint_->set_right_indicator == 1) && (CurrentWaypoint_->set_left_indicator == 1))
			{
				acs_pod_decriptor_.DiReqAcs = (uint8_t)4;
				acs_pod_decriptor_.InteriorLightsReqAcs = 10;
				acs_pod_decriptor_.HeadlampReqAcs = 2;
			}

			if (CurrentWaypoint_->set_horn == 0)
			{
				acs_pod_decriptor_.HornReqAcs = false;
			}
			else if (CurrentWaypoint_->set_horn == 1)
			{
				acs_pod_decriptor_.HornReqAcs = true;
				acs_pod_information_publisher_.publish(acs_pod_decriptor_);
			}

			std::cout << CurrentWaypoint_->audio_file << std::endl;

			if ((string1.compare(CurrentWaypoint_->audio_file) != 0) && (CurrentWaypoint_->set_play_audio == 1))
			{
				QString audio_file_string = QString::fromUtf8((CurrentWaypoint_->audio_file).c_str());
				qprocess_audio.startDetached("play", QStringList() << audio_file_string);
			}
			else
			{
				// Do nothing
			}
			ROS_INFO("Planner settings properly defined ");

			if (virtual_pause_)
			{
				move_base_action_.cancelAllGoals();
				ros::Rate loop_rate(20);
				while(ros::ok() && virtual_pause_)
				{
					ros::spinOnce();  // process incoming messages
					isNavigating_.data = false;
					is_pod_navigating_publisher_.publish(isNavigating_);
        			loop_rate.sleep();
				}

			}
			else
			{
				isNavigating_.data = true;
				is_pod_navigating_publisher_.publish(isNavigating_);
			}

			if (CurrentWaypoint_->pause_time == 0)
			{

			}
			else if (CurrentWaypoint_->pause_time == 4)
			{
				move_base_action_.cancelAllGoals();
				ROS_INFO("PAUSE TIME ""CONDITION!!!!1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				std::cout << "pause_release" << pause_button_release_.data << std::endl;
				ros::Duration(CurrentWaypoint_->pause_time).sleep();  // sleep for the required number of time
				char key = 0;
				pause_button_release_.data = 0;
				do
				{
					pause_set_.data = true;
					pause_set_publisher_.publish(pause_set_);
					std::cout << "pause_release message / key press;" << pause_button_release_.data << std::endl;
					boost::this_thread::sleep(boost::posix_time::milliseconds(50));
					ros::spinOnce();
				} while (pause_button_release_.data == 0);
				pause_button_release_.data = 0;
				pause_set_.data = false;
				pause_set_publisher_.publish(pause_set_);
			}
			else if (CurrentWaypoint_->pause_time == 1)
			{
				move_base_action_.cancelAllGoals();
				ROS_INFO("PAUSE TIME ""CONDITION!!!!1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				std::cout << "pause_release" << pause_button_release_.data << std::endl;
				ros::Duration(CurrentWaypoint_->pause_time).sleep();  // sleep for the required number of time
				char key = 0;
				pause_button_release_.data = 0;
				do
				{
					pause_set_.data = true;
					pause_set_publisher_.publish(pause_set_);
					std::cout << "pause_release message / key press;" << pause_button_release_.data << std::endl;
					boost::this_thread::sleep(boost::posix_time::milliseconds(50));
					// ros::spinOnce();
					#ifdef VEH_TYPE_ADT3
					if (sds_nav_mode == 9.0)
					{
						crab_height_.data = CurrentWaypoint_->crab_wheel_height_request;
						crab_height_publisher_.publish(crab_height_);
					}
					#endif
					sleep();
				} while (pause_button_release_.data == 0);
				pause_button_release_.data = 0;
				pause_set_.data = false;
				pause_set_publisher_.publish(pause_set_);
			}
			else if (CurrentWaypoint_->pause_time == 20)
			{
				move_base_action_.cancelAllGoals();
				ROS_INFO("PAUSE TIME ""CONDITION!!!!2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				std::cout << "pause_release loading" << pause_button_release_.data << std::endl;
				ros::Duration(CurrentWaypoint_->pause_time).sleep();  // sleep for the required number of time
				char key = 0;
				pause_button_release_.data = 0;
				do
				{
					pause_set_.data = true;
					pause_set_publisher_.publish(pause_set_);
					std::cout << "pause_release ;" << pause_button_release_.data << std::endl;
					boost::this_thread::sleep(boost::posix_time::milliseconds(50));
					ros::spinOnce();
				} while (pause_button_release_.data == 0);
				pause_button_release_.data = 0;
				pause_set_.data = false;
				pause_set_publisher_.publish(pause_set_);
			}
			else if (CurrentWaypoint_->pause_time == 21)
			{
				move_base_action_.cancelAllGoals();
				ROS_INFO("PAUSE TIME ""CONDITION!!!!2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				std::cout << "pause_release loading" << pause_button_release_.data << std::endl;
				ros::Duration(CurrentWaypoint_->pause_time).sleep();  // sleep for the required number of time
				char key = 0;
				pause_button_release_.data = 0;
				do
				{
					pause_set_.data = true;
					pause_set_publisher_.publish(pause_set_);
					std::cout << "pause_release ;" << pause_button_release_.data << std::endl;
					boost::this_thread::sleep(boost::posix_time::milliseconds(50));
					ros::spinOnce();
				} while (pause_button_release_.data == 0);
				pause_button_release_.data = 0;
				pause_set_.data = false;
				pause_set_publisher_.publish(pause_set_);
			}
			else if ((CurrentWaypoint_->pause_time > 2) & (CurrentWaypoint_->pause_time < 19))
			{
				move_base_action_.cancelAllGoals();
				ROS_INFO("PAUSE TIME ""CONDITION!!!!3!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				std::cout << "pause_release timed" << pause_button_release_.data << std::endl;
				ros::Duration(CurrentWaypoint_->pause_time).sleep();  // sleep for the required number of time
				char key = 0;
				pause_button_release_.data = 0;
				pause_set_.data = false;
				pause_set_publisher_.publish(pause_set_);
			}
			else
			{
				// DO NOTHING
			}

			if(!ListWaypoints_.empty())
			{
				sendGoal(std::next(CurrentWaypoint_)->waypoint_pose);
				
			}
			return true;


		}
		else
		{
			ROS_INFO("RequestPlannerSettings FAILED checking status. Problaby no reply on time !!! ");
			//      return false; // We take this out and move it down to the bottom of the function.
		}
		ROS_INFO("Number of Retries: %d ", counterRetries);

		counterRetries++;
	} while ((!successServiceCall) && (counterRetries < MaxNumberTries));

	return false;  // Since it has not met all the other conditions.
}

int navigator_interative_waypoints::getClosestWaypointToCurrentPosition(void)
{
	tf::StampedTransform EstimatedPose;
	if (!getRobotPosGL(EstimatedPose))
	{
		ROS_INFO("FAILED TO LOOK UP TRANSFORM, DON'T KNOW CLOSEST WP");
		return 0;
	}
	else
	{
		int index = 0;
		for (auto it = ListWaypoints_.begin(); it != ListWaypoints_.end(); it++, index++)
		{
			double Xrobot = EstimatedPose.getOrigin().getX();
			double Yrobot = EstimatedPose.getOrigin().getY();
			double ErrorPosition =
					sqrt(pow((Xrobot - it->waypoint_pose.position.x), 2) + pow((Yrobot - it->waypoint_pose.position.y), 2));

			double ErrorYaw =
					ecl::wrap_angle(tf::getYaw(EstimatedPose.getRotation()) - tf::getYaw(it->waypoint_pose.orientation));

			if ((ErrorPosition < ErrorPosDetectingInitPoint_) && (fabs(ErrorYaw) < ErrorHeadingDetectingInitPoint_))
				break;
		}
		return index;
	}
}

void navigator_interative_waypoints::sendGoal(geometry_msgs::Pose Pose)
{
	move_base_msgs::MoveBaseGoal move_base_goal;
	move_base_goal.target_pose.header.stamp = ros::Time::now();
	move_base_goal.target_pose.header.frame_id = world_frame_;
	move_base_goal.target_pose.pose.position = Pose.position;
	move_base_goal.target_pose.pose.orientation = Pose.orientation;

	ROS_INFO_STREAM("New Point sent!!! -->   " << move_base_goal.target_pose.pose.position << std::endl << move_base_goal.target_pose.pose.orientation);

	move_base_action_.sendGoal(move_base_goal);
}

bool navigator_interative_waypoints::isReadyToMoveNextWaypoint(void)
{
	std::stringstream str_stream; 
	tf::StampedTransform EstimatedPose;

	if (!getRobotPosGL(EstimatedPose))
	{
		ROS_INFO("FAILED TO LOOKUP TRANSFORM, CAN'T MOVE TO NEXT WAYPOINT");
		return false;

	}
	else
	{
		geometry_msgs::Pose GoalPose = std::next(CurrentWaypoint_)->waypoint_pose;
		if (msg_For_sds_goal_achieved_ != -1)
		{
			if (msg_For_sds_goal_achieved_ == CurrentWaypoint_->index)
			{
				// side drive system SDS has met the goal while crabbing
				return true;
			}
		}

		if((msg_For_ride_height_front_ < min_value_ride_height_front_) || (msg_For_ride_height_rear_ < min_value_ride_height_rear_))  // SPB we are on crabbing wheels
		{
			// don't allow planner to say we achieved goal when we are crabbing, as sds does that instead
			return false;
		}
		//added to stop navigator bumping the next wp index if we are intending to crab there
#ifdef defined(VEH_TYPE_ADT3) || defined(VEH_TYPE_ACA1)
		if(sds_nav_mode != 0) { //we are crabbing mode
			ROS_INFO("TEMP_DEBUG: [BLOCK 4] SDS in crabbing mode - BLOCKING TRANSITION");
			return false;
		}
#endif
		// if (std::next(CurrentWaypoint_) != ListWaypoints_.end()) 
		// {
		// 	if((std::next(CurrentWaypoint_)->av_drive_mode == 0)) 
		// 	{
		// 			return false;
		// 	}
		// }

		str_stream << " msg_For_ride_height_front_: " << msg_For_ride_height_front_ << " msg_For_ride_height_rear_ " << msg_For_ride_height_rear_ 
		<< " CurrentWaypoint_->crab_wheel_height_request : " << CurrentWaypoint_->crab_wheel_height_request  
		<< " CurrentWaypoint_->av_drive_mode  " << CurrentWaypoint_->av_drive_mode
		<< " CurrentWaypoint_->index  " << CurrentWaypoint_->index
		<< " min_value_ride_height_front_  " << min_value_ride_height_front_
		<< " min_value_ride_height_rear_  " << min_value_ride_height_rear_ 
		<< " cross  " << cross 
		<< " msg_For_manual_state_  " << msg_For_manual_state_ ;
		debug_string_3_msg_.data = str_stream.str();
		debug_string_3_publisher_.publish(debug_string_3_msg_);

		double Xrobot = EstimatedPose.getOrigin().getX();
		double Yrobot = EstimatedPose.getOrigin().getY();

		// Calculate theta in radians
		angle_of_pod = tf::getYaw(EstimatedPose.getRotation());
		// std::cout << "ANGLE OF THE POD****************************" << angle_of_pod << std::endl;
		pod_angle_in_radians = angle_of_pod * 3.14 / 180.0;

		point2_.x = std::round((Xrobot + 20 * cos(angle_of_pod + 1.571)));
		point2_.y = std::round((Yrobot + 20 * sin(angle_of_pod + 1.571)));
		points_for_Waypoint_Skipping_rviz_visualization_.points.push_back(point2_);

		point1_.x = std::round((Xrobot - 20 * cos(angle_of_pod + 1.571)));
		;
		point1_.y = std::round((Yrobot - 20 * sin(angle_of_pod + 1.571)));
		;
		points_for_Waypoint_Skipping_rviz_visualization_.points.push_back(point1_);

		line_visualization_points_publisher_.publish(points_for_Waypoint_Skipping_rviz_visualization_);

		dxc = std::round((GoalPose.position.x)) - point1_.x;
		dyc = std::round((GoalPose.position.y)) - point1_.y;

		dx1 = point2_.x - point1_.x;
		dy1 = point2_.y - point1_.y;

		cross = dxc * dy1 - dyc * dx1;

		double ErrorPosition = sqrt(pow((Xrobot - GoalPose.position.x), 2) + pow((Yrobot - GoalPose.position.y), 2));
		double ErrorYaw = ecl::wrap_angle(tf::getYaw(EstimatedPose.getRotation()) - tf::getYaw(GoalPose.orientation));

		if ((cross <= 2) && (cross >= -2) && (msg_For_manual_state_ == 6))
		{
			move_base_action_.cancelAllGoals();
			sendGoal(std::next(CurrentWaypoint_)->waypoint_pose);
			return true;
		}
		else
		{
			// if(CurrentWaypoint_->av_drive_mode == 1) {
			// 	return false;
			// }
			// 	str_stream << " msg_For_ride_height_front_: " << msg_For_ride_height_front_ << " msg_For_ride_height_rear_ " << msg_For_ride_height_rear_ 
			// 	<< " CurrentWaypoint_->crab_wheel_height_request : " << CurrentWaypoint_->crab_wheel_height_request  
			// 	<< " CurrentWaypoint_->av_drive_mode  " << CurrentWaypoint_->av_drive_mode
			// 	<< " CurrentWaypoint_->index  " << CurrentWaypoint_->index
			// 	<< " min_value_ride_height_front_  " << min_value_ride_height_front_
			// 	<< " min_value_ride_height_rear_  " << min_value_ride_height_rear_ 
			// 	<< " cross  " << cross 
			// 	<< " msg_For_manual_state_  " << msg_For_manual_state_ ;
			// 	debug_string_3_msg_.data = str_stream.str();
			// 	debug_string_3_publisher_.publish(debug_string_3_msg_);
			if(((CurrentWaypoint_->crab_wheel_height_request == 100) && (CurrentWaypoint_->av_drive_mode == 0)) &&
			((msg_For_ride_height_front_ < min_value_ride_height_front_) || (msg_For_ride_height_rear_ < min_value_ride_height_rear_)))  // SPB not the right height yet
				{
					return false;
			}
			else
			{

				ROS_INFO_STREAM("navigator_interative_waypoints::isReadyToMoveNextWaypoint --> ErrorPosition: "<< ErrorPosition << " ErrorYaw: " << ErrorYaw << std::endl);
				if (CurrentWaypoint_->av_drive_mode == 0)
				{
					// str_stream << " Case 1: ErrorPosition: "<< ErrorPosition << " ErrorYaw: " << ErrorYaw << " DistanceFromGoalToMoveNextWaypoint_: " << DistanceFromGoalToMoveNextWaypoint_ ;
					// debug_string_msg_.data = str_stream.str();
					// debug_string_publisher_.publish(debug_string_msg_);

					//add a speed fatcor here??
					double  speed_factor_acceptance = 1.0;
					if((ErrorPosition < (DistanceFromGoalToMoveNextWaypoint_ * speed_factor_acceptance) && (fabs(ErrorYaw) < (HeadingFromGoalToMoveNextWaypoint_ * speed_factor_acceptance)))) 
						// if ((cross <= 2) && (cross >= -2) && (msg_For_manual_state_ == 11)) //auto mode only
					{
						// str_stream << " cross: " << cross << " msg_For_manual_state_ " << msg_For_manual_state_;
						// debug_string_msg_.data = str_stream.str();
						// debug_string_publisher_.publish(debug_string_msg_);

						move_base_action_.cancelAllGoals();
						sendGoal(std::next(CurrentWaypoint_)->waypoint_pose);
						return true;
					}

					// return (
					// 		(ErrorPosition < DistanceFromGoalToMoveNextWaypoint_) &&
					// 		(fabs(ErrorYaw) < HeadingFromGoalToMoveNextWaypoint_));
				}
				else
				{

				}
			}
		}
		return (false);
	}

}

bool navigator_interative_waypoints::getRobotPosGL(tf::StampedTransform &robot_gl)
{
    try
    {
        tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
        return true; // Success
    }
    catch (tf::TransformException& e)
    {
        ROS_WARN_STREAM("tf::TransformException: " << e.what());
        return false; // Failed, robot_gl not modified
    }
}

void navigator_interative_waypoints::sleep(void)
{
	rate_.sleep();
	ros::spinOnce();
}

void navigator_interative_waypoints::run(void)
{
	std::stringstream str_stream_1;
	
	ros::Rate rate(20);  // 10 Hz = 100ms per loop

	while (ros::ok())
	{
		if (isNavigating_.data == true)
		{
			crab_height_.data = CurrentWaypoint_->crab_wheel_height_request;
			// if ((sds_nav_mode == 7.0) || (sds_nav_mode == 6.0))
			// {
			// 	crab_height_.data = 0.0;
			// }

			current_drive_mode_.data = CurrentWaypoint_->av_drive_mode;
			//SPB nned to allow drive mode to get us to raise when met wp to raise
			if (std::prev(CurrentWaypoint_) != ListWaypoints_.begin())
			{
				if (((CurrentWaypoint_->crab_wheel_height_request == 100) && (CurrentWaypoint_->av_drive_mode == 0)) &&
					 ((std::prev(CurrentWaypoint_)->crab_wheel_height_request == 0) && (std::prev(CurrentWaypoint_)->av_drive_mode == 1))) 
				{
					if ((msg_For_ride_height_front_ < min_value_ride_height_front_) || (msg_For_ride_height_rear_ < min_value_ride_height_rear_))  // SPB not the right height yet
					{
						current_drive_mode_.data = true;
					}
				}
			}
			current_drive_mode_publisher_.publish(current_drive_mode_);

			//SPB
			str_stream_1 << "current_drive_mode_.data: " << current_drive_mode_.data << " CurrentWaypoint_->av_drive_mode " << CurrentWaypoint_->av_drive_mode;
			debug_string_msg_.data = str_stream_1.str();
			debug_string_publisher_.publish(debug_string_msg_);

			if (isReadyToMoveNextWaypoint())
			{
				std::stringstream str_stream;
				str_stream << " CurrentWaypoint_->index" << CurrentWaypoint_->index << " std::next(CurrentWaypoint_)->index" << std::next(CurrentWaypoint_)->index;
				CurrentWaypoint_ = std::next(CurrentWaypoint_);
				debug_string_2_msg_.data = str_stream.str();
				debug_string_2_publisher_.publish(debug_string_2_msg_);

				if (std::next(CurrentWaypoint_) == ListWaypoints_.end())
				{
					current_waypoint_index_.data = CurrentWaypoint_->index;
					current_waypoint_index_publisher_.publish(current_waypoint_index_);

					// Use ROS sleep instead of std::this_thread::sleep_until
					move_base_action_.cancelAllGoals();
					isNavigating_.data = false;
					is_pod_navigating_publisher_.publish(isNavigating_);

					ros::Duration(0.70).sleep();

					/*	isNavigating_.data = false;
          is_pod_navigating_publisher_.publish(isNavigating_);
					 */
					end_of_route_ = true;
					std_msgs::Bool end_of_route;
					end_of_route.data = end_of_route_;
					end_of_route_current_route_publisher_.publish(end_of_route);
					ros::spinOnce();
				}

				ROS_INFO("Position of CurrentWaypoint_ %d", std::distance(ListWaypoints_.begin(), CurrentWaypoint_));
				if (!sendWaypointPlanner())
				{
					isNavigating_.data = false;
				}
			}

			is_pod_navigating_publisher_.publish(isNavigating_);
			ros::spinOnce();
		}

		crab_height_publisher_.publish(crab_height_);
		ros::spinOnce();  // Process ROS callbacks

		rate.sleep();  // Ensure loop runs at a controlled rate
		ros::spinOnce();
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "navigator_interative_waypoints");
	navigator_interative_waypoints w_nav;
	//    ros::spin();
	w_nav.run();

	return 0;
}
