# waypoints_path_planner
Interactive waypoints replacement for autonomous vehicle navigation.

## Overview

The waypoints_path_planner (AutoWpp - short for "Autonomous Waypoint Path Planner") is a ROS package that provides an interactive RViz panel for creating, editing, and managing waypoint-based navigation for autonomous vehicles.

## Key Features

- **Interactive Waypoint Management**: Create and edit waypoints directly in RViz
- **Multi-Lane Support**: Main lane, left lane, and right lane waypoint sets
- **Route Planning**: Define and manage multi-segment routes
- **No-Go Zones**: Temporal dynamic zones for obstacle avoidance
- **System Monitoring**: Real-time vehicle and sensor status display
- **Navigation Control**: Start, stop, and monitor autonomous navigation

## Architecture

This package has been refactored to separate concerns:

- **StatusMonitoringPanel**: System status visualization (battery, sensors, navigation state)
- **VisualizationManager**: Interactive markers and RViz visualization
- **WaypointsPathPlanner**: Core planning logic and route management  
- **Navigator**: Waypoint-following navigation execution (separate node)

For detailed architecture information, see [ARCHITECTURE.md](ARCHITECTURE.md).

For refactoring details and migration notes, see [REFACTORING.md](REFACTORING.md).

## Quick Start

### Prerequisites

- ROS (tested on ROS Melodic/Noetic)
- Required dependencies (see `package.xml`)
- Vehicle-specific message definitions (ADT3, ACA1, POD, or STL2)

### Building

```bash
cd ~/catkin_ws/src
git clone <repository-url>
cd ~/catkin_ws
catkin_make -Dveh_type=ADT3  # or POD, ACA1, STL2
source devel/setup.bash
```

### Running

```bash
roslaunch waypoints_path_planner waypoints_path_planner.launch
```

Then in RViz, add the "WaypointsPathPlanner" panel from: Panels → Add New Panel

## Routing System Usage 

### Workflow Overview 

1. **Unlock the Edit Button**: Begin by unlocking the edit button. This enables editing features in the routing system interface.

2. **Create a Route File**: Click on the Create Route File button. A file dialog will open. Create a new file and name it (e.g., example.yaml). This becomes your default route file. You can create multiple route files. Each route file can contain multiple segments.

3. **Load Route File**: If you've already created a route file, use the Load Route File button. This loads the existing route file for editing or testing.

4. **Set Route File**: Click the Set Route File button. If no route has been set yet, it allows you to define route segments. Click the Add Route button to add segments. As you add them, you'll see new segments listed. Use the route index scroll to view and manage different segment combinations. Currently, up to four segments can be selected per route.

5. **Close the Route Management Panel**: The panel that opens when you press Set Route File is called the Route Management Panel. After setting segments, you can close it using the Close Route button.

6. **Run Routes**: You can test individual routes using:
   - Run Route 1
   - Run Route 2
   - Run Route 3
   - Run Route 4
   
   Ensure your vehicle is at the correct starting position for each route before starting.

## Documentation

- **[ARCHITECTURE.md](ARCHITECTURE.md)**: System architecture and component descriptions
- **[REFACTORING.md](REFACTORING.md)**: Refactoring guide and migration notes
- **[package.xml](package.xml)**: ROS package dependencies

## Contributing

When contributing to this repository, please maintain the separation of concerns:

- Status monitoring changes go in `StatusMonitoringPanel`
- Visualization changes go in `VisualizationManager`
- Core planning changes go in `WaypointsPathPlanner`
- Navigation execution changes go in `navigator_interactive_waypoint`

## License

TODO: Add license information

## Support

For issues, questions, or contributions, please contact the maintainers.

