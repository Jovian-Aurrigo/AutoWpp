/*
 * status_monitoring_panel.cpp
 *
 * Implementation of system status monitoring and display functionality.
 * This module is responsible for all system status visualization including
 * battery, sensors, navigation state, and vehicle systems monitoring.
 *
 *  Created on: Oct 2024
 *      Purpose: Separation of visualization/status from core waypoint planning logic
 */

#include "waypoints_path_planner/status_monitoring_panel.h"
#include <QFont>

namespace waypoints_path_planner
{

StatusMonitoringPanel::StatusMonitoringPanel(ros::NodeHandle& nh, QGridLayout* layout)
    : nh_(nh), grid_layout_(layout), pod_state_(0), boot_safe_(false),
      last_imu_state_on_(false), last_lidar_state_on_(false),
      // Initialize all QMovie pointers to nullptr
      movie_battery_charging_(nullptr), movie_battery_full_(nullptr),
      movie_battery_half_(nullptr), movie_battery_quarter_(nullptr),
      movie_battery_three_quarter_(nullptr), movie_battery_depleted_(nullptr),
      movie_battery_unknown_(nullptr), movie_battery_full_plugged_in_(nullptr),
      movie_autonomy_on_(nullptr), movie_autonomy_off_(nullptr),
      movie_autonomy_unknown_(nullptr), movie_dms_on_(nullptr),
      movie_dms_off_(nullptr), movie_dms_unknown_(nullptr),
      movie_dms_uninitialised_(nullptr), movie_dtp_on_(nullptr),
      movie_dtp_off_(nullptr), movie_dtp_unknown_(nullptr),
      movie_imu_on_(nullptr), movie_imu_off_(nullptr),
      movie_bbr_full_(nullptr), movie_bbr_no_drive_in_bay_(nullptr),
      movie_time_machine_ready_(nullptr), movie_time_machine_writing_(nullptr)
{
}

StatusMonitoringPanel::~StatusMonitoringPanel()
{
    // Cleanup Qt objects - delete is safe for nullptr in C++
    delete movie_battery_charging_;
    delete movie_battery_full_;
    delete movie_battery_half_;
    delete movie_battery_quarter_;
    delete movie_battery_three_quarter_;
    delete movie_battery_depleted_;
    delete movie_battery_unknown_;
    delete movie_battery_full_plugged_in_;
    delete movie_autonomy_on_;
    delete movie_autonomy_off_;
    delete movie_autonomy_unknown_;
    delete movie_dms_on_;
    delete movie_dms_off_;
    delete movie_dms_unknown_;
    delete movie_dms_uninitialised_;
    delete movie_dtp_on_;
    delete movie_dtp_off_;
    delete movie_dtp_unknown_;
    delete movie_imu_on_;
    delete movie_imu_off_;
    delete movie_bbr_full_;
    delete movie_bbr_no_drive_in_bay_;
    delete movie_time_machine_ready_;
    delete movie_time_machine_writing_;
}

QMovie* StatusMonitoringPanel::createValidatedMovie(const std::string& param_name)
{
    std::string file_path;
    if (nh_.getParam(param_name, file_path)) {
        QMovie* movie = new QMovie(QString::fromStdString(file_path));
        if (movie && movie->isValid()) {
            return movie;
        } else {
            ROS_WARN("Failed to load or invalid movie file for parameter '%s': %s", 
                     param_name.c_str(), file_path.c_str());
            delete movie;
            return nullptr;
        }
    }
    return nullptr;
}

void StatusMonitoringPanel::initialize()
{
    initializeLabels();
    initializeMovies();
    initializePixmaps();
    addLabelsToLayout();
    initializeSubscribers();
    initializeTimers();
}

void StatusMonitoringPanel::initializeLabels()
{
    // Battery status label
    label_battery_ = new QLabel;
    label_battery_->setAlignment(Qt::AlignLeft);
    label_battery_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_battery_->setWordWrap(true);
    label_battery_->setFont(QFont("FreeSans", 72));
    label_battery_->setVisible(true);
    
    // Autonomy enable/disable label
    label_autonomy_ = new QLabel;
    label_autonomy_->setAlignment(Qt::AlignLeft);
    label_autonomy_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_autonomy_->setWordWrap(true);
    label_autonomy_->setFont(QFont("FreeSans", 72));
    label_autonomy_->setVisible(true);
    
    // DMS status label
    label_dms_ = new QLabel;
    label_dms_->setAlignment(Qt::AlignLeft);
    label_dms_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_dms_->setWordWrap(true);
    label_dms_->setFont(QFont("FreeSans", 72));
    label_dms_->setVisible(true);
    
    // DTP status label
    label_dtp_ = new QLabel;
    label_dtp_->setAlignment(Qt::AlignLeft);
    label_dtp_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_dtp_->setWordWrap(true);
    label_dtp_->setFont(QFont("FreeSans", 72));
    label_dtp_->setVisible(true);
    
    // IMU status label
    label_imu_ = new QLabel;
    label_imu_->setAlignment(Qt::AlignLeft);
    label_imu_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_imu_->setWordWrap(true);
    label_imu_->setFont(QFont("FreeSans", 72));
    label_imu_->setVisible(true);
    
    // BBR status label
    label_bbr_ = new QLabel;
    label_bbr_->setAlignment(Qt::AlignLeft);
    label_bbr_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_bbr_->setWordWrap(true);
    label_bbr_->setFont(QFont("FreeSans", 72));
    label_bbr_->setVisible(true);
    
    // Time Machine status label
    label_time_machine_ = new QLabel;
    label_time_machine_->setAlignment(Qt::AlignLeft);
    label_time_machine_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_time_machine_->setWordWrap(true);
    label_time_machine_->setFont(QFont("FreeSans", 72));
    label_time_machine_->setVisible(true);
    
    // Lidar status label
    label_lidar_ = new QLabel;
    label_lidar_->setAlignment(Qt::AlignLeft);
    label_lidar_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_lidar_->setWordWrap(true);
    label_lidar_->setFont(QFont("FreeSans", 72));
    label_lidar_->setVisible(true);
    
    // Navigation status label
    label_navigation_ = new QLabel;
    label_navigation_->setAlignment(Qt::AlignLeft);
    label_navigation_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_navigation_->setWordWrap(true);
    label_navigation_->setFont(QFont("FreeSans", 72));
    label_navigation_->setVisible(true);
    
    // Covariance status label
    label_covariance_ = new QLabel;
    label_covariance_->setAlignment(Qt::AlignLeft);
    label_covariance_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_covariance_->setWordWrap(true);
    label_covariance_->setFont(QFont("FreeSans", 72));
    label_covariance_->setVisible(true);
    
    // System Monitor status label
    label_system_monitor_ = new QLabel;
    label_system_monitor_->setAlignment(Qt::AlignLeft);
    label_system_monitor_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_system_monitor_->setWordWrap(true);
    label_system_monitor_->setFont(QFont("FreeSans", 72));
    label_system_monitor_->setVisible(true);
    
    // Loading arms status label
    label_loading_arms_ = new QLabel;
    label_loading_arms_->setAlignment(Qt::AlignLeft);
    label_loading_arms_->setStyleSheet("font-weight: bold;color: black; font-size:14pt");
    label_loading_arms_->setWordWrap(true);
    label_loading_arms_->setFont(QFont("FreeSans", 72));
    label_loading_arms_->setVisible(true);
}

void StatusMonitoringPanel::initializeMovies()
{
    // Load and validate movie files from parameters
    
    // Battery movies
    movie_battery_unknown_ = createValidatedMovie("/battery_unknown_file");
    movie_battery_charging_ = createValidatedMovie("/battery_charging_file");
    movie_battery_depleted_ = createValidatedMovie("/battery_depleted_file");
    movie_battery_full_ = createValidatedMovie("/battery_full_file");
    movie_battery_half_ = createValidatedMovie("/battery_half_file");
    movie_battery_quarter_ = createValidatedMovie("/battery_quarter_file");
    movie_battery_three_quarter_ = createValidatedMovie("/battery_three_quarter_file");
    movie_battery_full_plugged_in_ = createValidatedMovie("/battery_charger_plugged_in_file");
    
    // Set default movie for battery
    if (movie_battery_unknown_) {
        label_battery_->setMovie(movie_battery_unknown_);
        movie_battery_unknown_->start();
    }
    
    // Autonomy movies
    movie_autonomy_unknown_ = createValidatedMovie("/autonomy_switch_unknown_file");
    movie_autonomy_on_ = createValidatedMovie("/autonomy_switch_on_file");
    movie_autonomy_off_ = createValidatedMovie("/autonomy_switch_off_file");
    
    if (movie_autonomy_unknown_) {
        label_autonomy_->setMovie(movie_autonomy_unknown_);
        movie_autonomy_unknown_->start();
    }
    
    // DMS movies
    movie_dms_unknown_ = createValidatedMovie("/dms_unknown_file");
    movie_dms_uninitialised_ = createValidatedMovie("/dms_not_initialised_file");
    movie_dms_off_ = createValidatedMovie("/dms_not_armed_file");
    movie_dms_on_ = createValidatedMovie("/dms_armed_file");
    
    if (movie_dms_unknown_) {
        label_dms_->setMovie(movie_dms_unknown_);
        movie_dms_unknown_->start();
    }
    
    // DTP movies
    movie_dtp_unknown_ = createValidatedMovie("/dtp_unknown_file");
    movie_dtp_on_ = createValidatedMovie("/dtp_on_file");
    movie_dtp_off_ = createValidatedMovie("/dtp_off_file");
    
    // IMU movies
    movie_imu_on_ = createValidatedMovie("/imu_connected_file");
    movie_imu_off_ = createValidatedMovie("/imu_not_connected_file");
    
    if (movie_imu_off_) {
        label_imu_->setMovie(movie_imu_off_);
        movie_imu_off_->start();
    }
    
    // BBR movies
    movie_bbr_full_ = createValidatedMovie("/bbr_full_file");
    movie_bbr_no_drive_in_bay_ = createValidatedMovie("/bbr_no_drive_in_bay_file");
    
    // Time Machine movies
    movie_time_machine_ready_ = createValidatedMovie("/time_machine_ready_file");
    movie_time_machine_writing_ = createValidatedMovie("/time_machine_writing_file");
}
}

void StatusMonitoringPanel::initializePixmaps()
{
    std::string file_path;
    
    // BBR pixmaps
    if (nh_.getParam("/bbr_unknown_file", file_path)) {
        bbr_unknown_.load(QString::fromStdString(file_path));
        label_bbr_->setPixmap(bbr_unknown_);
    }
    if (nh_.getParam("/bbr_ok_0_file", file_path)) {
        bbr_ok_0_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/bbr_ok_1_file", file_path)) {
        bbr_ok_1_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/bbr_ok_2_file", file_path)) {
        bbr_ok_2_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/bbr_ok_3_file", file_path)) {
        bbr_ok_3_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/bbr_ok_4_file", file_path)) {
        bbr_ok_4_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/bbr_stop_file", file_path)) {
        bbr_stop_.load(QString::fromStdString(file_path));
    }
    
    // Time Machine pixmaps
    if (nh_.getParam("/time_machine_error_file", file_path)) {
        time_machine_error_.load(QString::fromStdString(file_path));
        label_time_machine_->setPixmap(time_machine_error_);
    }
    
    // Lidar pixmaps
    if (nh_.getParam("/lidar_not_connected_file", file_path)) {
        lidar_off_.load(QString::fromStdString(file_path));
        label_lidar_->setPixmap(lidar_off_);
    }
    if (nh_.getParam("/lidar_connected_file", file_path)) {
        lidar_on_.load(QString::fromStdString(file_path));
    }
    
    // Navigation pixmaps
    if (nh_.getParam("/navigation_unknown_file", file_path)) {
        navigation_unknown_.load(QString::fromStdString(file_path));
        label_navigation_->setPixmap(navigation_unknown_);
    }
    if (nh_.getParam("/navigation_paused_file", file_path)) {
        navigation_paused_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/navigation_stopped_file", file_path)) {
        navigation_stopped_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/navigation_running_file", file_path)) {
        navigation_running_.load(QString::fromStdString(file_path));
    }
    
    // Covariance pixmaps
    if (nh_.getParam("/covariance_unknown_file", file_path)) {
        covariance_unknown_.load(QString::fromStdString(file_path));
        label_covariance_->setPixmap(covariance_unknown_);
    }
    if (nh_.getParam("/covariance_ok_file", file_path)) {
        covariance_good_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/covariance_error_file", file_path)) {
        covariance_error_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/covariance_poor_file", file_path)) {
        covariance_poor_.load(QString::fromStdString(file_path));
    }
    
    // System monitor pixmaps
    if (nh_.getParam("/ads_temp_unknown_file", file_path)) {
        ads_temp_unknown_.load(QString::fromStdString(file_path));
        label_system_monitor_->setPixmap(ads_temp_unknown_);
    }
    if (nh_.getParam("/ads_temp_ok", file_path)) {
        ads_temp_ok_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/cpu_temp_high", file_path)) {
        cpu_temp_high_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/gpu_temp_high_file", file_path)) {
        gpu_temp_high_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/ads_temp_high_file", file_path)) {
        ads_temp_high_.load(QString::fromStdString(file_path));
    }
    
    // Loading arms pixmaps
    if (nh_.getParam("/loading_arms_unknown_file", file_path)) {
        loading_arms_unknown_.load(QString::fromStdString(file_path));
        label_loading_arms_->setPixmap(loading_arms_unknown_);
    }
    if (nh_.getParam("/loading_arms_deployed_file", file_path)) {
        loading_arms_deployed_.load(QString::fromStdString(file_path));
    }
    if (nh_.getParam("/loading_arms_retracted_file", file_path)) {
        loading_arms_retracted_.load(QString::fromStdString(file_path));
    }
}

void StatusMonitoringPanel::addLabelsToLayout()
{
    if (!grid_layout_) return;
    
    // Add labels to grid layout at appropriate positions
    grid_layout_->addWidget(label_battery_, 18, 0, 1, 1);
    grid_layout_->addWidget(label_autonomy_, 18, 1, 1, 1);
    grid_layout_->addWidget(label_dms_, 18, 2, 1, 1);
    grid_layout_->addWidget(label_dtp_, 18, 3, 1, 1);
    grid_layout_->addWidget(label_imu_, 20, 0, 1, 1);
    grid_layout_->addWidget(label_bbr_, 20, 1, 1, 1);
    grid_layout_->addWidget(label_time_machine_, 20, 2, 1, 1);
    grid_layout_->addWidget(label_lidar_, 20, 3, 1, 1);
    grid_layout_->addWidget(label_navigation_, 22, 0, 1, 1);
    grid_layout_->addWidget(label_covariance_, 22, 1, 1, 1);
    grid_layout_->addWidget(label_system_monitor_, 22, 2, 1, 1);
    grid_layout_->addWidget(label_loading_arms_, 22, 3, 1, 1);
}

void StatusMonitoringPanel::initializeSubscribers()
{
    // Subscribe to relevant topics
    vehicle_position_error_sub_ = nh_.subscribe("/vehicle_position_error", 1, 
        &StatusMonitoringPanel::vehiclePositionErrorCallback, this);
    is_av_navigating_sub_ = nh_.subscribe("/isPodNavigating", 10, 
        &StatusMonitoringPanel::isAvNavigatingCallback, this);
    
    // Vehicle-specific topics
    std::string topic_prefix;
#ifdef VEH_TYPE_ACA1
    topic_prefix = "/aca1/";
#elif defined(VEH_TYPE_POD)
    topic_prefix = "/pod/";
#elif defined(VEH_TYPE_STL2)
    topic_prefix = "/stl2/";
#elif defined(VEH_TYPE_ADT3)
    topic_prefix = "/adt3/";
#endif
    
    pod_to_acs_core_sub_ = nh_.subscribe(topic_prefix + "av_to_ads_core", 10, 
        &StatusMonitoringPanel::podToAcsCoreCallback, this);
    pod_to_acs_aux_sub_ = nh_.subscribe(topic_prefix + "av_to_ads_aux", 10, 
        &StatusMonitoringPanel::podToAcsAuxCallback, this);
        
    dms_status_sub_ = nh_.subscribe("/DM_Speed_Reduction_Factor_Message", 10, 
        &StatusMonitoringPanel::dmsStatusCallback, this);
    micro_imu_sub_ = nh_.subscribe("/gx5/imu/data", 1000, 
        &StatusMonitoringPanel::microImuCallback, this);
    merged_cloud_sub_ = nh_.subscribe("/points_concat", 100, 
        &StatusMonitoringPanel::mergedCloudCallback, this);
    time_machine_sub_ = nh_.subscribe("/Timemachine_Info", 10, 
        &StatusMonitoringPanel::timeMachineCallback, this);
    system_monitor_sub_ = nh_.subscribe("/aurrigo_System_Monitor", 10, 
        &StatusMonitoringPanel::systemMonitorCallback, this);
    bbr_sub_ = nh_.subscribe("/BlackBox_Info", 10, 
        &StatusMonitoringPanel::bbrCallback, this);
}

void StatusMonitoringPanel::initializeTimers()
{
    // Initialize timers for status checks (1 Hz by default)
    timer_vehicle_position_error_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerVehiclePositionError, this));
    timer_navigation_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerNavigation, this));
    timer_battery_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerBattery, this));
    timer_dms_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerDms, this));
    timer_imu_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerImu, this));
    timer_bbr_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerBbr, this));
    timer_time_machine_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerTimeMachine, this));
    timer_system_monitor_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerSystemMonitor, this));
    timer_loading_arms_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerLoadingArms, this));
    timer_lidar_ = nh_.createTimer(ros::Duration(1.0), 
        boost::bind(&StatusMonitoringPanel::timerLidar, this));
        
    // Start all timers
    timer_vehicle_position_error_.start();
    timer_navigation_.start();
    timer_battery_.start();
    timer_dms_.start();
    timer_imu_.start();
    timer_bbr_.start();
    timer_time_machine_.start();
    timer_system_monitor_.start();
    timer_loading_arms_.start();
    timer_lidar_.start();
}

// Callback implementations
void StatusMonitoringPanel::vehiclePositionErrorCallback(const std_msgs::Float32& msg)
{
    timer_vehicle_position_error_.start();
    time_from_vehicle_position_error_ = ros::Time::now();
    
    if (msg.data <= 1.0) {
        label_covariance_->setPixmap(covariance_good_);
    } else if (msg.data >= 3.0) {
        label_covariance_->setPixmap(covariance_error_);
    } else {
        label_covariance_->setPixmap(covariance_poor_);
    }
}

void StatusMonitoringPanel::isAvNavigatingCallback(const std_msgs::Bool& msg)
{
    timer_navigation_.start();
    time_from_navigation_ = ros::Time::now();
    
    if (msg.data) {
        label_navigation_->setPixmap(navigation_running_);
    } else {
        label_navigation_->setPixmap(navigation_stopped_);
    }
}

void StatusMonitoringPanel::podToAcsCoreCallback(const VehicleAVToADSCOREMsg& msg)
{
    timer_battery_.start();
    time_from_pod_to_acs_core_ = ros::Time::now();
    pod_state_ = static_cast<uint8_t>(msg.AvState);
}

void StatusMonitoringPanel::podToAcsAuxCallback(const VehicleAVToADSAUXMsg& msg)
{
    timer_battery_.start();
    time_from_pod_to_acs_aux_ = ros::Time::now();
    
    // Update battery status based on state of charge
    if ((pod_state_ == 13) && (msg.StateOfCharge < 95)) {
        label_battery_->setMovie(movie_battery_charging_);
        movie_battery_charging_->start();
    } else if ((msg.StateOfCharge >= 95) && (pod_state_ == 13)) {
        label_battery_->setMovie(movie_battery_full_plugged_in_);
        movie_battery_full_plugged_in_->start();
    } else if (msg.StateOfCharge >= 76) {
        label_battery_->setMovie(movie_battery_full_);
        movie_battery_full_->start();
    } else if ((msg.StateOfCharge > 50) && (msg.StateOfCharge <= 75)) {
        label_battery_->setMovie(movie_battery_three_quarter_);
        movie_battery_three_quarter_->start();
    } else if ((msg.StateOfCharge > 25) && (msg.StateOfCharge <= 50)) {
        label_battery_->setMovie(movie_battery_half_);
        movie_battery_half_->start();
    } else if ((msg.StateOfCharge > 5) && (msg.StateOfCharge <= 25)) {
        label_battery_->setMovie(movie_battery_quarter_);
        movie_battery_quarter_->start();
    } else if (msg.StateOfCharge <= 5) {
        label_battery_->setMovie(movie_battery_depleted_);
        movie_battery_depleted_->start();
    }
    
    // Update autonomy status
    if (msg.AutonomyEnableSwitchState) {
        label_autonomy_->setMovie(movie_autonomy_on_);
        movie_autonomy_on_->start();
    } else {
        label_autonomy_->setMovie(movie_autonomy_off_);
        movie_autonomy_off_->start();
    }
}

void StatusMonitoringPanel::dmsStatusCallback(const VehicleDMSStatusMsg& msg)
{
    timer_dms_.start();
    time_from_dms_ = ros::Time::now();
    
    if (msg.state == 0) {
        label_dms_->setMovie(movie_dms_uninitialised_);
        movie_dms_uninitialised_->start();
    } else if (msg.state > 0) {
        label_dms_->setMovie(movie_dms_on_);
        movie_dms_on_->start();
    }
}

void StatusMonitoringPanel::microImuCallback(const sensor_msgs::Imu& msg)
{
    timer_imu_.start();
    time_from_imu_ = ros::Time::now();
    
    label_imu_->setMovie(movie_imu_on_);
    movie_imu_on_->start();
}

void StatusMonitoringPanel::mergedCloudCallback(const sensor_msgs::PointCloud2& msg)
{
    timer_lidar_.start();
    time_from_lidar_ = ros::Time::now();
    
    label_lidar_->setPixmap(lidar_on_);
}

void StatusMonitoringPanel::timeMachineCallback(const VehicleTimeMachineMsg& msg)
{
    timer_time_machine_.start();
    time_from_time_machine_ = ros::Time::now();
    
    if (msg.TimeMachineSystemState == 0) {
        label_time_machine_->setMovie(movie_time_machine_ready_);
        movie_time_machine_ready_->start();
    } else if (msg.TimeMachineSystemState == 2) {
        label_time_machine_->setMovie(movie_time_machine_writing_);
        movie_time_machine_writing_->start();
    } else {
        label_time_machine_->setPixmap(time_machine_error_);
    }
}

void StatusMonitoringPanel::systemMonitorCallback(const VehicleSystemMonitorMsg& msg)
{
    timer_system_monitor_.start();
    time_from_system_monitor_ = ros::Time::now();
    
    if (msg.GPUTemperature > 85) {
        label_system_monitor_->setPixmap(gpu_temp_high_);
    } else if (msg.CPU0TempCurrent > 70) {
        label_system_monitor_->setPixmap(cpu_temp_high_);
    } else {
        label_system_monitor_->setPixmap(ads_temp_ok_);
    }
}

void StatusMonitoringPanel::bbrCallback(const VehicleBlackboxRecorderMsg& msg)
{
    timer_bbr_.start();
    time_from_bbr_ = ros::Time::now();
    
    boot_safe_ = msg.BootSafe;
    
    if ((msg.BlackBoxSystemState == 0) && (msg.PercentFull <= 25)) {
        label_bbr_->setPixmap(bbr_ok_0_);
    } else if ((msg.BlackBoxSystemState == 0) && ((msg.PercentFull > 25) && (msg.PercentFull <= 50))) {
        label_bbr_->setPixmap(bbr_ok_1_);
    } else if ((msg.BlackBoxSystemState == 0) && ((msg.PercentFull > 50) && (msg.PercentFull <= 75))) {
        label_bbr_->setPixmap(bbr_ok_2_);
    } else if ((msg.BlackBoxSystemState == 0) && ((msg.PercentFull > 75) && (msg.PercentFull <= 95))) {
        label_bbr_->setPixmap(bbr_ok_4_);
    } else if ((msg.BlackBoxSystemState == 4) || (msg.BlackBoxSystemState == 7)) {
        label_bbr_->setPixmap(bbr_stop_);
    } else if (msg.BlackBoxSystemState == 5) {
        label_bbr_->setMovie(movie_bbr_no_drive_in_bay_);
        movie_bbr_no_drive_in_bay_->start();
    } else if (msg.BlackBoxSystemState == 6) {
        label_bbr_->setMovie(movie_bbr_full_);
        movie_bbr_full_->start();
    }
}

// Timer callback implementations
void StatusMonitoringPanel::timerVehiclePositionError()
{
    ros::Duration elapsed = ros::Time::now() - time_from_vehicle_position_error_;
    if (elapsed.sec > 1) {
        timer_vehicle_position_error_.stop();
    }
}

void StatusMonitoringPanel::timerNavigation()
{
    ros::Duration elapsed = ros::Time::now() - time_from_navigation_;
    if (elapsed.sec > 1) {
        label_navigation_->setPixmap(navigation_unknown_);
        timer_navigation_.stop();
    }
}

void StatusMonitoringPanel::timerBattery()
{
    ros::Duration elapsed_core = ros::Time::now() - time_from_pod_to_acs_core_;
    ros::Duration elapsed_aux = ros::Time::now() - time_from_pod_to_acs_aux_;
    
    if ((elapsed_core.sec > 2) || (elapsed_aux.sec > 2)) {
        label_battery_->setMovie(movie_battery_unknown_);
        movie_battery_unknown_->start();
        label_autonomy_->setMovie(movie_autonomy_unknown_);
        movie_autonomy_unknown_->start();
        timer_battery_.stop();
    }
}

void StatusMonitoringPanel::timerDms()
{
    ros::Duration elapsed = ros::Time::now() - time_from_dms_;
    if (elapsed.sec > 1) {
        label_dms_->setMovie(movie_dms_unknown_);
        movie_dms_unknown_->start();
        timer_dms_.stop();
    }
}

void StatusMonitoringPanel::timerImu()
{
    ros::Duration elapsed = ros::Time::now() - time_from_imu_;
    if (elapsed.sec > 1) {
        if (last_imu_state_on_ != false) {
            label_imu_->setMovie(movie_imu_off_);
            movie_imu_off_->start();
            last_imu_state_on_ = false;
        }
        timer_imu_.stop();
    } else {
        if (last_imu_state_on_ != true) {
            label_imu_->setMovie(movie_imu_on_);
            movie_imu_on_->start();
            last_imu_state_on_ = true;
        }
    }
}

void StatusMonitoringPanel::timerBbr()
{
    ros::Duration elapsed = ros::Time::now() - time_from_bbr_;
    if (elapsed.sec > 1) {
        label_bbr_->setPixmap(bbr_unknown_);
        timer_bbr_.stop();
    }
}

void StatusMonitoringPanel::timerTimeMachine()
{
    ros::Duration elapsed = ros::Time::now() - time_from_time_machine_;
    if (elapsed.sec > 1) {
        label_time_machine_->setPixmap(time_machine_error_);
        timer_time_machine_.stop();
    }
}

void StatusMonitoringPanel::timerSystemMonitor()
{
    ros::Duration elapsed = ros::Time::now() - time_from_system_monitor_;
    if (elapsed.sec > 1) {
        label_system_monitor_->setPixmap(ads_temp_unknown_);
        timer_system_monitor_.stop();
    }
}

void StatusMonitoringPanel::timerLoadingArms()
{
    ros::Duration elapsed = ros::Time::now() - time_from_loading_arms_;
    if (elapsed.sec > 1) {
        label_loading_arms_->setPixmap(loading_arms_unknown_);
        timer_loading_arms_.stop();
    }
}

void StatusMonitoringPanel::timerLidar()
{
    ros::Duration elapsed = ros::Time::now() - time_from_lidar_;
    if (elapsed.sec > 1) {
        if (last_lidar_state_on_ != false) {
            label_lidar_->setPixmap(lidar_off_);
            last_lidar_state_on_ = false;
        }
        timer_lidar_.stop();
    } else {
        if (last_lidar_state_on_ != true) {
            label_lidar_->setPixmap(lidar_on_);
            last_lidar_state_on_ = true;
        }
    }
}

} // namespace waypoints_path_planner
