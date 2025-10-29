/*
 * status_monitoring_panel.h
 *
 * Handles system status monitoring and display for the waypoint path planner.
 * This class manages all status-related UI components including:
 * - Battery status
 * - Autonomy enable/disable
 * - DMS (Driver Monitoring System) status
 * - DTP (Data Transfer Protocol) status
 * - IMU status
 * - BlackBox Recorder status
 * - Time Machine status
 * - System Monitor (temperature, etc.)
 * - Loading arms status
 * - Lidar status
 * - Navigation status
 * - Covariance/localization quality
 *
 *  Created on: Oct 2024
 *      Purpose: Separation of visualization/status from core waypoint planning logic
 */

#ifndef WAYPOINTS_PATH_PLANNER_INCLUDE_STATUS_MONITORING_PANEL_H_
#define WAYPOINTS_PATH_PLANNER_INCLUDE_STATUS_MONITORING_PANEL_H_

#include <ros/ros.h>
#include <QLabel>
#include <QMovie>
#include <QPixmap>
#include <QGridLayout>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

// Vehicle-specific message includes
#ifdef VEH_TYPE_ACA1
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
#elif defined(VEH_TYPE_POD)
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
#elif defined(VEH_TYPE_ADT3)
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
#else
 #error "Vehicle type not defined. Please define VEH_TYPE_ACA1, VEH_TYPE_POD, VEH_TYPE_STL2, or VEH_TYPE_ADT3"
#endif

namespace waypoints_path_planner
{

/**
 * @brief StatusMonitoringPanel - Manages system status displays and monitoring
 * 
 * This class handles all UI components related to system status monitoring including
 * battery, sensors, navigation state, and vehicle systems. It subscribes to relevant
 * topics and updates the display accordingly.
 */
class StatusMonitoringPanel
{
public:
    /**
     * @brief Constructor
     * @param nh ROS NodeHandle for topic subscription
     * @param layout Qt grid layout to add status widgets to
     */
    StatusMonitoringPanel(ros::NodeHandle& nh, QGridLayout* layout);
    
    /**
     * @brief Destructor
     */
    ~StatusMonitoringPanel();

    /**
     * @brief Initialize status panel UI components and ROS subscribers
     */
    void initialize();

private:
    // ROS communication
    ros::NodeHandle nh_;
    
    // Subscribers for system status
    ros::Subscriber vehicle_position_error_sub_;
    ros::Subscriber is_av_navigating_sub_;
    ros::Subscriber pod_to_acs_core_sub_;
    ros::Subscriber pod_to_acs_aux_sub_;
    ros::Subscriber dms_status_sub_;
    ros::Subscriber micro_imu_sub_;
    ros::Subscriber merged_cloud_sub_;
    ros::Subscriber time_machine_sub_;
    ros::Subscriber system_monitor_sub_;
    ros::Subscriber bbr_sub_;
    
    // Timers for status checks
    ros::Timer timer_vehicle_position_error_;
    ros::Timer timer_navigation_;
    ros::Timer timer_battery_;
    ros::Timer timer_dms_;
    ros::Timer timer_imu_;
    ros::Timer timer_bbr_;
    ros::Timer timer_time_machine_;
    ros::Timer timer_system_monitor_;
    ros::Timer timer_loading_arms_;
    ros::Timer timer_lidar_;
    
    // UI Components - Labels
    QLabel* label_battery_;
    QLabel* label_autonomy_;
    QLabel* label_dms_;
    QLabel* label_dtp_;
    QLabel* label_imu_;
    QLabel* label_bbr_;
    QLabel* label_time_machine_;
    QLabel* label_lidar_;
    QLabel* label_navigation_;
    QLabel* label_covariance_;
    QLabel* label_system_monitor_;
    QLabel* label_loading_arms_;
    
    // UI Components - Movies/Animations
    QMovie* movie_battery_charging_;
    QMovie* movie_battery_full_;
    QMovie* movie_battery_half_;
    QMovie* movie_battery_quarter_;
    QMovie* movie_battery_three_quarter_;
    QMovie* movie_battery_depleted_;
    QMovie* movie_battery_unknown_;
    QMovie* movie_battery_full_plugged_in_;
    QMovie* movie_autonomy_on_;
    QMovie* movie_autonomy_off_;
    QMovie* movie_autonomy_unknown_;
    QMovie* movie_dms_on_;
    QMovie* movie_dms_off_;
    QMovie* movie_dms_unknown_;
    QMovie* movie_dms_uninitialised_;
    QMovie* movie_dtp_on_;
    QMovie* movie_dtp_off_;
    QMovie* movie_dtp_unknown_;
    QMovie* movie_imu_on_;
    QMovie* movie_imu_off_;
    QMovie* movie_bbr_full_;
    QMovie* movie_bbr_no_drive_in_bay_;
    QMovie* movie_time_machine_ready_;
    QMovie* movie_time_machine_writing_;
    
    // UI Components - Pixmaps/Images
    QPixmap bbr_unknown_;
    QPixmap bbr_ok_0_, bbr_ok_1_, bbr_ok_2_, bbr_ok_3_, bbr_ok_4_;
    QPixmap bbr_stop_;
    QPixmap time_machine_error_;
    QPixmap lidar_on_, lidar_off_;
    QPixmap navigation_unknown_, navigation_paused_, navigation_stopped_, navigation_running_;
    QPixmap covariance_unknown_, covariance_good_, covariance_error_, covariance_poor_;
    QPixmap ads_temp_unknown_, ads_temp_ok_, cpu_temp_high_, gpu_temp_high_, ads_temp_high_;
    QPixmap loading_arms_unknown_, loading_arms_deployed_, loading_arms_retracted_;
    
    // Status tracking
    ros::Time time_from_vehicle_position_error_;
    ros::Time time_from_navigation_;
    ros::Time time_from_pod_to_acs_core_;
    ros::Time time_from_pod_to_acs_aux_;
    ros::Time time_from_dms_;
    ros::Time time_from_imu_;
    ros::Time time_from_bbr_;
    ros::Time time_from_time_machine_;
    ros::Time time_from_system_monitor_;
    ros::Time time_from_loading_arms_;
    ros::Time time_from_lidar_;
    
    uint8_t pod_state_;
    bool boot_safe_;
    
    QGridLayout* grid_layout_;
    
    // Callback functions
    void vehiclePositionErrorCallback(const std_msgs::Float32& msg);
    void isAvNavigatingCallback(const std_msgs::Bool& msg);
    void podToAcsCoreCallback(const VehicleAVToADSCOREMsg& msg);
    void podToAcsAuxCallback(const VehicleAVToADSAUXMsg& msg);
    void dmsStatusCallback(const VehicleDMSStatusMsg& msg);
    void microImuCallback(const sensor_msgs::Imu& msg);
    void mergedCloudCallback(const sensor_msgs::PointCloud2& msg);
    void timeMachineCallback(const VehicleTimeMachineMsg& msg);
    void systemMonitorCallback(const VehicleSystemMonitorMsg& msg);
    void bbrCallback(const VehicleBlackboxRecorderMsg& msg);
    
    // Timer callbacks
    void timerVehiclePositionError();
    void timerNavigation();
    void timerBattery();
    void timerDms();
    void timerImu();
    void timerBbr();
    void timerTimeMachine();
    void timerSystemMonitor();
    void timerLoadingArms();
    void timerLidar();
    
    // Initialization helpers
    void initializeLabels();
    void initializeMovies();
    void initializePixmaps();
    void initializeSubscribers();
    void initializeTimers();
    void addLabelsToLayout();
};

} // namespace waypoints_path_planner

#endif // WAYPOINTS_PATH_PLANNER_INCLUDE_STATUS_MONITORING_PANEL_H_
