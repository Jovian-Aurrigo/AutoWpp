/*
 * visualization_manager.cpp
 *
 * Implementation of visualization and interactive marker management.
 * This module is responsible for creating, updating, and managing all
 * RViz visualization elements including waypoints, no-go zones, and trajectories.
 *
 *  Created on: Oct 2024
 *      Purpose: Separation of visualization from core waypoint planning logic
 */

#include "waypoints_path_planner/visualization_manager.h"
#include <tf/transform_datatypes.h>

namespace waypoints_path_planner
{

VisualizationManager::VisualizationManager(ros::NodeHandle& nh)
    : nh_(nh)
{
}

VisualizationManager::~VisualizationManager()
{
    if (marker_server_) {
        marker_server_->clear();
    }
}

void VisualizationManager::initialize()
{
    // Create interactive marker server
    marker_server_.reset(new interactive_markers::InteractiveMarkerServer("waypoints_markers", "", false));
    
    // Create publishers
    waypoint_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/waypoint_markers", 1, true);
    nogozones_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/nogozone_markers", 1, true);
    trajectory_pub_ = nh_.advertise<visualization_msgs::Marker>("/trajectory_visualization", 1, true);
    vehicle_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vehicle_pose_visualization", 1, true);
    
    // Apply changes to interactive marker server
    marker_server_->applyChanges();
    
    ROS_INFO("VisualizationManager initialized");
}

void VisualizationManager::createWaypointMarker(const geometry_msgs::Pose& pose,
                                               const std::string& name,
                                               int index,
                                               const std::string& lane_type)
{
    if (!marker_server_) {
        ROS_ERROR("Marker server not initialized");
        return;
    }
    
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = name;
    int_marker.description = "Waypoint " + std::to_string(index);
    int_marker.pose = pose;
    
    // Create a sphere marker for the waypoint
    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.5;
    sphere_marker.scale.y = 0.5;
    sphere_marker.scale.z = 0.5;
    
    // Color based on lane type
    if (lane_type == "main") {
        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 1.0;
        sphere_marker.color.b = 0.0;
    } else if (lane_type == "left") {
        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 0.0;
        sphere_marker.color.b = 1.0;
    } else if (lane_type == "right") {
        sphere_marker.color.r = 1.0;
        sphere_marker.color.g = 0.0;
        sphere_marker.color.b = 0.0;
    }
    sphere_marker.color.a = 0.8;
    
    // Create control for the marker
    visualization_msgs::InteractiveMarkerControl marker_control;
    marker_control.always_visible = true;
    marker_control.markers.push_back(sphere_marker);
    int_marker.controls.push_back(marker_control);
    
    // Add 6-DOF control
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_plane";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);
    
    // Insert the marker
    marker_server_->insert(int_marker);
    marker_server_->applyChanges();
    
    // Track the marker name
    waypoint_marker_names_.push_back(name);
    
    ROS_DEBUG("Created waypoint marker: %s at index %d", name.c_str(), index);
}

void VisualizationManager::createNoGoZoneMarker(const geometry_msgs::Pose& pose,
                                               double radius,
                                               const std::string& name,
                                               int version)
{
    if (!marker_server_) {
        ROS_ERROR("Marker server not initialized");
        return;
    }
    
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = name;
    int_marker.description = "No-Go Zone (v" + std::to_string(version) + ")";
    int_marker.pose = pose;
    
    // Create a cylinder marker for the no-go zone
    visualization_msgs::Marker cylinder_marker;
    cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
    cylinder_marker.scale.x = radius * 2.0;  // Diameter
    cylinder_marker.scale.y = radius * 2.0;  // Diameter
    cylinder_marker.scale.z = 0.1;          // Height (thin disk)
    
    // Color based on version
    if (version == 1) {
        cylinder_marker.color.r = 1.0;
        cylinder_marker.color.g = 0.5;
        cylinder_marker.color.b = 0.0;
    } else if (version == 2) {
        cylinder_marker.color.r = 1.0;
        cylinder_marker.color.g = 0.0;
        cylinder_marker.color.b = 0.5;
    } else {
        cylinder_marker.color.r = 1.0;
        cylinder_marker.color.g = 0.0;
        cylinder_marker.color.b = 0.0;
    }
    cylinder_marker.color.a = 0.5;
    
    // Create control for the marker
    visualization_msgs::InteractiveMarkerControl marker_control;
    marker_control.always_visible = true;
    marker_control.markers.push_back(cylinder_marker);
    int_marker.controls.push_back(marker_control);
    
    // Add move control
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_plane";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);
    
    // Insert the marker
    marker_server_->insert(int_marker);
    marker_server_->applyChanges();
    
    // Track the marker name
    nogozone_marker_names_.push_back(name);
    
    ROS_DEBUG("Created no-go zone marker: %s with radius %.2f", name.c_str(), radius);
}

void VisualizationManager::updateWaypointMarker(const std::string& name,
                                               const geometry_msgs::Pose& new_pose)
{
    if (!marker_server_) {
        ROS_ERROR("Marker server not initialized");
        return;
    }
    
    visualization_msgs::InteractiveMarker int_marker;
    if (marker_server_->get(name, int_marker)) {
        int_marker.pose = new_pose;
        marker_server_->setPose(name, new_pose);
        marker_server_->applyChanges();
        ROS_DEBUG("Updated waypoint marker: %s", name.c_str());
    } else {
        ROS_WARN("Waypoint marker not found: %s", name.c_str());
    }
}

void VisualizationManager::removeWaypointMarker(const std::string& name)
{
    if (!marker_server_) {
        ROS_ERROR("Marker server not initialized");
        return;
    }
    
    if (marker_server_->erase(name)) {
        marker_server_->applyChanges();
        
        // Remove from tracking
        auto it = std::find(waypoint_marker_names_.begin(), waypoint_marker_names_.end(), name);
        if (it != waypoint_marker_names_.end()) {
            waypoint_marker_names_.erase(it);
        }
        
        ROS_DEBUG("Removed waypoint marker: %s", name.c_str());
    } else {
        ROS_WARN("Could not remove waypoint marker: %s", name.c_str());
    }
}

void VisualizationManager::removeNoGoZoneMarker(const std::string& name)
{
    if (!marker_server_) {
        ROS_ERROR("Marker server not initialized");
        return;
    }
    
    if (marker_server_->erase(name)) {
        marker_server_->applyChanges();
        
        // Remove from tracking
        auto it = std::find(nogozone_marker_names_.begin(), nogozone_marker_names_.end(), name);
        if (it != nogozone_marker_names_.end()) {
            nogozone_marker_names_.erase(it);
        }
        
        ROS_DEBUG("Removed no-go zone marker: %s", name.c_str());
    } else {
        ROS_WARN("Could not remove no-go zone marker: %s", name.c_str());
    }
}

void VisualizationManager::clearAllWaypointMarkers()
{
    if (!marker_server_) {
        ROS_ERROR("Marker server not initialized");
        return;
    }
    
    for (const auto& name : waypoint_marker_names_) {
        marker_server_->erase(name);
    }
    
    marker_server_->applyChanges();
    waypoint_marker_names_.clear();
    
    ROS_INFO("Cleared all waypoint markers");
}

void VisualizationManager::clearAllNoGoZoneMarkers()
{
    if (!marker_server_) {
        ROS_ERROR("Marker server not initialized");
        return;
    }
    
    for (const auto& name : nogozone_marker_names_) {
        marker_server_->erase(name);
    }
    
    marker_server_->applyChanges();
    nogozone_marker_names_.clear();
    
    ROS_INFO("Cleared all no-go zone markers");
}

void VisualizationManager::publishTrajectory(const std::vector<geometry_msgs::Pose>& trajectory,
                                             float color_r, float color_g, float color_b)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "trajectory";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.scale.x = 0.1;  // Line width
    line_strip.color.r = color_r;
    line_strip.color.g = color_g;
    line_strip.color.b = color_b;
    line_strip.color.a = 1.0;
    
    for (const auto& pose : trajectory) {
        geometry_msgs::Point point;
        point.x = pose.position.x;
        point.y = pose.position.y;
        point.z = pose.position.z;
        line_strip.points.push_back(point);
    }
    
    trajectory_pub_.publish(line_strip);
    ROS_DEBUG("Published trajectory with %zu points", trajectory.size());
}

void VisualizationManager::publishVehiclePose(const geometry_msgs::Pose& pose)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = pose;
    
    vehicle_pose_pub_.publish(pose_stamped);
}

// Helper function implementations
visualization_msgs::Marker VisualizationManager::createSphereMarker(
    const geometry_msgs::Pose& pose,
    const std::string& ns,
    int id,
    float scale,
    float r, float g, float b, float a)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

visualization_msgs::Marker VisualizationManager::createCylinderMarker(
    const geometry_msgs::Pose& pose,
    const std::string& ns,
    int id,
    float radius,
    float height,
    float r, float g, float b, float a)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = radius * 2.0;
    marker.scale.y = radius * 2.0;
    marker.scale.z = height;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

visualization_msgs::Marker VisualizationManager::createArrowMarker(
    const geometry_msgs::Pose& pose,
    const std::string& ns,
    int id,
    float scale,
    float r, float g, float b, float a)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = scale;
    marker.scale.y = scale * 0.1;
    marker.scale.z = scale * 0.1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

} // namespace waypoints_path_planner
