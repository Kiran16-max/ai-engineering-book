---
id: module3-chapter4-nav2-path-planning
title: 'Module 3 – Chapter 4: Nav2 - Path Planning for Bipedal Humanoid Movement'
sidebar_label: 'Module 3 – Chapter 4: Nav2 Path Planning'
---

## Chapter Overview

Welcome to Chapter 4 of Module 3. This chapter is dedicated to one of the most critical aspects of autonomous humanoid robotics: **path planning**. We will be exploring **Nav2**, the second generation of the ROS Navigation Stack, and its application to bipedal humanoid robots.

Path planning is the process of finding a safe and efficient route for a robot to move from a starting point to a destination, while avoiding obstacles and respecting the robot's physical limitations. For bipedal humanoids, this is a particularly challenging task due to their inherent instability and complex kinematics.

In this chapter, you will learn about:
- The architecture and core components of Nav2.
- Fundamental path planning concepts, including global and local planning.
- How to integrate Nav2 with a ROS 2-based humanoid robot.
- Practical exercises to get you started with bipedal path planning.

By the end of this chapter, you will have a solid understanding of how to implement robust navigation for your humanoid robot projects.

## Nav2 Architecture

Nav2 is a powerful and flexible navigation framework for ROS 2. It is designed to be highly configurable and extensible, allowing it to be adapted to a wide variety of robots and environments.

The main components of the Nav2 stack are:
- **BT Navigator**: The Behavior Tree Navigator is the high-level orchestrator of Nav2. It uses Behavior Trees to define complex navigation logic.
- **Planners**: Nav2 includes a variety of path planning algorithms, both for global and local planning. These planners are responsible for generating safe and efficient trajectories for the robot.
- **Controllers**: The controller plugins are responsible for executing the planned trajectories, taking into account the robot's dynamics and kinematics.
- **Costmaps**: Nav2 uses costmaps to represent the environment and to identify obstacles. There are two types of costmaps: the global costmap for long-term planning, and the local costmap for short-term obstacle avoidance.
- **Lifecycle Manager**: This component is responsible for managing the lifecycle of the Nav2 nodes, ensuring that they start and stop in a coordinated manner.

## Path Planning Concepts

### Global and Local Planning

Path planning is typically divided into two stages: **global planning** and **local planning**.

- **Global Planning**: The global planner is responsible for finding an optimal path from the robot's current location to the goal, taking into account the static environment represented by the global costmap. The output of the global planner is a high-level path that the robot should follow.
- **Local Planning**: The local planner is responsible for generating feasible control commands to follow the global plan, while avoiding dynamic obstacles and respecting the robot's kinematic and dynamic constraints. The local planner operates on the local costmap, which is a smaller, rolling window around the robot.

### Costmaps

A **costmap** is a 2D or 3D grid that represents the robot's environment. Each cell in the grid has a value that corresponds to the cost of traversing that cell. Obstacles are assigned a high cost, while free space is assigned a low cost.

Nav2 uses two costmaps:
- The **global costmap** is a large, static map that is used by the global planner.
- The **local costmap** is a smaller, rolling window around the robot that is used by the local planner. The local costmap is updated in real-time with sensor data to account for dynamic obstacles.

### Bipedal Humanoid Constraints

Path planning for bipedal humanoids is particularly challenging due to their unique constraints:
- **Stability**: Bipedal robots are inherently unstable and require careful control to maintain balance. The path planner must generate smooth trajectories that do not destabilize the robot.
- **Kinematics**: The robot's kinematic constraints, such as joint limits and self-collision, must be taken into account.
- **Gait Generation**: The path planner must be synchronized with the robot's gait generator to ensure that the planned motion is physically achievable.

## Integration with ROS 2

Nav2 is tightly integrated with ROS 2, and communicates with other nodes via topics, services, and actions.

- **Topics**: Nav2 subscribes to sensor data (e.g., laser scans, point clouds) and publishes the planned path and control commands.
- **Services**: Nav2 provides services for setting goals, clearing costmaps, and other administrative tasks.
- **Actions**: The main way to interact with Nav2 is through the `navigate_to_pose` action, which allows you to send a goal to the robot and receive feedback on its progress.

## Practical Example: Bipedal Path Planning with Nav2

In this example, we will configure Nav2 to work with a simulated bipedal humanoid robot.

**1. Installation and Configuration**

First, make sure you have Nav2 installed:
```bash
sudo apt-get install ros-foxy-nav2-bringup
```

Next, we need to create a configuration file for Nav2. This file will specify which plugins to use, and how to configure them. Here is a minimal example:

```yaml
# my_nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: odom
    robot_base_frame: base_link
    odom_topic: /odom
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
```

**2. Launching Nav2**

Now, we can launch Nav2 with our configuration file:

```bash
ros2 launch nav2_bringup nav2_bringup_launch.py use_sim_time:=True params_file:=/path/to/my_nav2_params.yaml
```

**3. Sending a Goal**

Once Nav2 is running, we can send a goal to the robot using the `ros2 action send_goal` command:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Nav2 will then start planning a path to the goal and publishing control commands to the robot.

## Module Connection

This chapter builds upon the knowledge you have gained in the previous modules:
- **Module 1: ROS 2 Fundamentals**: Your understanding of ROS 2 nodes, topics, services, and actions is essential for working with Nav2.
- **Module 2: The Digital Twin**: You can use the digital twin of your humanoid robot to test and validate your Nav2 configuration in simulation before deploying it on the physical robot.

This chapter prepares you for the final capstone project, where you will be expected to implement a complete navigation solution for a bipedal humanoid robot.

## Assessment Tasks

1.  **Configure Nav2 for a Custom Robot**:
    -   Take the URDF of a simple bipedal robot (e.g., a two-legged robot with basic joints).
    -   Create a complete Nav2 configuration for this robot, including costmap settings, planner choices, and controller parameters.
    -   Justify your choice of planners and controllers for this specific robot.

2.  **Obstacle Avoidance Challenge**:
    -   Set up a simulated environment with static and dynamic obstacles.
    -   Use Nav2 to navigate your bipedal robot through the environment, avoiding all obstacles.
    -   Experiment with different costmap settings and sensor inputs to improve the robot's obstacle avoidance capabilities.

3.  **Behavior Tree Customization**:
    -   Create a custom Behavior Tree for Nav2 that implements a specific navigation behavior (e.g., "patrol a designated area," "follow a person").
    -   Test your custom Behavior Tree in simulation and document its performance.
---
