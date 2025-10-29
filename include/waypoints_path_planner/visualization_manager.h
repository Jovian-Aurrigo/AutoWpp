/*
 * visualization_manager.h
 *
 * Handles interactive marker visualization for the waypoint path planner.
 * This class manages all visualization-related functionality including:
 * - Interactive waypoint markers
 * - No-go zone markers
 * - Vehicle pose visualization
 * - Route visualization
 * - Trajectory display
 *
 *  Created on: Oct 2024
 *      Purpose: Separation of visualization from core waypoint planning logic
 */

#ifndef WAYPOINTS_PATH_PLANNER_INCLUDE_VISUALIZATION_MANAGER_H_
#define WAYPOINTS_PATH_PLANNER_INCLUDE_VISUALIZATION_MANAGER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <vector>

namespace waypoints_path_planner
{

/**
 * @brief VisualizationManager - Manages interactive markers and visualization
 * 
 * This class handles all RViz visualization including interactive markers for
 * waypoints, no-go zones, and other visual elements. It provides a clean
 * interface for creating, updating, and removing markers.
 */
class VisualizationManager
{
public:
    /**
     * @brief Constructor
     * @param nh ROS NodeHandle for topic publishing
     */
    explicit VisualizationManager(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~VisualizationManager();

    /**
     * @brief Initialize the visualization manager
     * Sets up interactive marker server and publishers
     */
    void initialize();

    /**
     * @brief Create an interactive waypoint marker
     * @param pose Pose of the waypoint
     * @param name Name/identifier for the marker
     * @param index Index of the waypoint in the sequence
     * @param lane_type Type of lane (main, left, right)
     */
    void createWaypointMarker(const geometry_msgs::Pose& pose, 
                             const std::string& name,
                             int index,
                             const std::string& lane_type = "main");

    /**
     * @brief Create a no-go zone marker
     * @param pose Center pose of the no-go zone
     * @param radius Radius of the no-go zone
     * @param name Name/identifier for the marker
     * @param version Version/type of no-go zone behavior
     */
    void createNoGoZoneMarker(const geometry_msgs::Pose& pose,
                             double radius,
                             const std::string& name,
                             int version = 0);

    /**
     * @brief Update an existing waypoint marker
     * @param name Name of the marker to update
     * @param new_pose New pose for the marker
     */
    void updateWaypointMarker(const std::string& name, 
                             const geometry_msgs::Pose& new_pose);

    /**
     * @brief Remove a waypoint marker
     * @param name Name of the marker to remove
     */
    void removeWaypointMarker(const std::string& name);

    /**
     * @brief Remove a no-go zone marker
     * @param name Name of the marker to remove
     */
    void removeNoGoZoneMarker(const std::string& name);

    /**
     * @brief Clear all waypoint markers
     */
    void clearAllWaypointMarkers();

    /**
     * @brief Clear all no-go zone markers
     */
    void clearAllNoGoZoneMarkers();

    /**
     * @brief Publish a trajectory visualization
     * @param trajectory Vector of poses representing the trajectory
     * @param color_r Red component (0-1)
     * @param color_g Green component (0-1)
     * @param color_b Blue component (0-1)
     */
    void publishTrajectory(const std::vector<geometry_msgs::Pose>& trajectory,
                          float color_r = 0.0, 
                          float color_g = 1.0, 
                          float color_b = 0.0);

    /**
     * @brief Publish vehicle pose visualization
     * @param pose Current vehicle pose
     */
    void publishVehiclePose(const geometry_msgs::Pose& pose);

private:
    ros::NodeHandle nh_;
    
    // Interactive marker server
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
    
    // Publishers
    ros::Publisher waypoint_markers_pub_;
    ros::Publisher nogozones_markers_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher vehicle_pose_pub_;
    
    // Marker tracking
    std::vector<std::string> waypoint_marker_names_;
    std::vector<std::string> nogozone_marker_names_;
    
    // Helper functions
    visualization_msgs::Marker createSphereMarker(const geometry_msgs::Pose& pose,
                                                  const std::string& ns,
                                                  int id,
                                                  float scale,
                                                  float r, float g, float b, float a);
    
    visualization_msgs::Marker createCylinderMarker(const geometry_msgs::Pose& pose,
                                                   const std::string& ns,
                                                   int id,
                                                   float radius,
                                                   float height,
                                                   float r, float g, float b, float a);
    
    visualization_msgs::Marker createArrowMarker(const geometry_msgs::Pose& pose,
                                                 const std::string& ns,
                                                 int id,
                                                 float scale,
                                                 float r, float g, float b, float a);
};

} // namespace waypoints_path_planner

#endif // WAYPOINTS_PATH_PLANNER_INCLUDE_VISUALIZATION_MANAGER_H_
