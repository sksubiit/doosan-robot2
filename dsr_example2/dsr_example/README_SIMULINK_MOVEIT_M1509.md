# Doosan M1509 Simulink to MoveIt 2 Guide

This document describes a minimal workflow to control the Doosan M1509 using:

- ROS 2 Jazzy
- the existing Doosan `dsr_bringup2` and `dsr_moveit2` packages
- the `dsr_example` package in this workspace
- a Simulink model that publishes `geometry_msgs/Pose`
- the `pose_to_moveit_bridge` script that forwards that pose into MoveIt 2

The goal is to keep the setup simple and explicit:

1. start the Doosan MoveIt stack in virtual mode or real mode
2. start the ROS 2 bridge script
3. trigger Simulink to publish one pose
4. MoveIt plans and executes the motion

## What This README Covers

- how to build and source the workspace
- how to run the M1509 in virtual mode
- how to run the M1509 on the real robot
- how the `pose_to_moveit_bridge` script works
- how to configure Simulink to publish a single pose
- how to verify the end-effector pose with TF
- how to avoid repeated goal spam from Simulink

## Package Used

This guide uses the existing package:

- `dsr_example`

The relevant script is:

- `dsr_example/simple/pose_to_moveit_bridge.py`

The script subscribes to:

- `/my_topic/simulink/target_pose`

and forwards the pose to MoveIt action server:

- `/move_action`

or, if the Doosan launch uses a namespace:

- `/<namespace>/move_action`

## Conceptual Summary

Simulink publishes a `geometry_msgs/Pose`.

That pose is not sent directly to the robot hardware.
It is first converted by the bridge into a MoveIt `MoveGroup` action goal.

The bridge uses:

- planning group: `manipulator`
- target link: `link_6`
- planning frame: usually `base_link`
- motion execution through MoveIt 2

In other words:

- Simulink provides the target pose
- ROS 2 receives the pose
- the bridge converts it into MoveIt format
- MoveIt handles planning and execution

## Important Frame Rule

The target pose must be expressed in the same frame as the bridge parameter `frame_id`.

If `frame_id := base_link`, then the published pose is the pose of `link_6` relative to `base_link`.

This is the recommended setup for this workspace because the MoveIt launch shows `base_link` as the planning frame.

## Current End-Effector Pose Reference

Use TF to read the current end-effector pose:

```bash
ros2 run tf2_ros tf2_echo base_link link_6
```

This prints the current pose of `link_6` expressed in `base_link`.

The returned values are the best starting point for choosing a new target pose.

## Workspace Setup

### 1. Source ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Source the Doosan workspace after building

If your workspace is already built, source it before running anything:

```bash
cd ~/ros2_ws
source install/setup.bash
```

If you rebuild, do it with:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DDRCF_VER=3
source install/setup.bash
```

### 3. Build only the example package when editing the bridge

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dsr_example
source install/setup.bash
```

## M1509 Virtual Mode

This is the easiest way to test the bridge first.

### Terminal 1: start Doosan MoveIt in virtual mode

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m1509
```

This starts:

- the Doosan controller interface
- MoveIt 2
- the emulator-backed virtual robot

### Terminal 2: start the pose bridge

Use the bridge with the same planning frame used by MoveIt:

```bash
ros2 run dsr_example pose_to_moveit_bridge --ros-args \
  -p action_name:=/move_action \
  -p pose_topic:=/my_topic/simulink/target_pose \
  -p group_name:=manipulator \
  -p link_name:=link_6 \
  -p frame_id:=base_link \
  -p orientation_enabled:=false
```

### Terminal 3: run Simulink and publish one pose

Start the Simulink model and publish a single `geometry_msgs/Pose` message to:

- `/my_topic/simulink/target_pose`

Recommended behavior:

- publish once
- wait for the result
- publish again only when you want a new motion

Do not continuously publish the same target at high rate unless you intentionally want repeated planning.

## M1509 Real Robot Mode

Use this once the virtual workflow is confirmed.

### Prerequisites

Before the robot is used:

- the robot controller must be powered on
- the teach pendant must be active
- there must be no warning or emergency stop condition
- the controller IP must be reachable from the workstation
- `rt_host` must match the controller IP

For the real robot in this workspace, the controller IP used in the issue notes is:

- `192.168.137.100`

### Terminal 1: start Doosan MoveIt in real mode

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=real \
  model:=m1509 \
  host:=192.168.137.100 \
  rt_host:=192.168.137.100
```

### Terminal 2: start the pose bridge

```bash
ros2 run dsr_example pose_to_moveit_bridge --ros-args \
  -p action_name:=/move_action \
  -p pose_topic:=/my_topic/simulink/target_pose \
  -p group_name:=manipulator \
  -p link_name:=link_6 \
  -p frame_id:=base_link \
  -p orientation_enabled:=false
```

### Terminal 3: run Simulink and publish one pose

Trigger the Simulink block once so that it publishes the target pose.

## Simulink Setup

The Simulink model should publish a `geometry_msgs/Pose` to:

- `/my_topic/simulink/target_pose`

Recommended pattern:

- generate the pose once
- publish once
- stop until the next command cycle

This keeps the robot from receiving a continuous stream of new goals.

### Why one-shot publishing matters

If Simulink publishes every sample time, MoveIt may continuously receive new target poses and replan repeatedly.

That can cause:

- repeated success/failure messages
- motion instability
- hard-to-read execution behavior
- awkward target changes while the robot is still moving

For a clean control cycle, publish a single pose per command.

## Example Safe Target Pose

A good first test is to stay close to the current home pose and use an identity quaternion.

From TF, the current pose is often around:

- position: `x = 0.000`, `y = 0.035`, `z = 1.053`
- orientation: `x = 0.0`, `y = 0.0`, `z = 0.0`, `w = 1.0`

A safe nearby test target is:

- position:
  - `x = 0.100`
  - `y = 0.035`
  - `z = 1.053`
- orientation:
  - `x = 0.0`
  - `y = 0.0`
  - `z = 0.0`
  - `w = 1.0`

If you want to keep orientation unchanged, use the identity quaternion.

## What the Bridge Does Internally

The bridge script does the following:

1. creates a ROS 2 node
2. creates a MoveIt `MoveGroup` action client
3. subscribes to the Simulink pose topic
4. normalizes the quaternion
5. builds a `PositionConstraint`
6. optionally builds an `OrientationConstraint`
7. builds a `MotionPlanRequest`
8. fills `group_name := manipulator`
9. sends the action goal to MoveIt
10. waits for the result and prints the MoveIt result code

## Bridge Parameters

The script exposes these parameters:

- `action_name`
- `pose_topic`
- `group_name`
- `link_name`
- `frame_id`
- `position_tolerance`
- `orientation_enabled`
- `orientation_tolerance`
- `max_velocity_scaling`
- `max_acceleration_scaling`
- `allowed_planning_time`

Suggested values for this workspace:

- `action_name := /move_action`
- `pose_topic := /my_topic/simulink/target_pose`
- `group_name := manipulator`
- `link_name := link_6`
- `frame_id := base_link`
- `position_tolerance := 0.01`
- `orientation_enabled := false`

## Example Launch Sequence

### Virtual mode

Terminal 1:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m1509
```

Terminal 2:

```bash
ros2 run dsr_example pose_to_moveit_bridge --ros-args \
  -p action_name:=/move_action \
  -p pose_topic:=/my_topic/simulink/target_pose \
  -p group_name:=manipulator \
  -p link_name:=link_6 \
  -p frame_id:=base_link \
  -p orientation_enabled:=false
```

Terminal 3:

- start the Simulink block
- publish one target pose

### Real mode

Terminal 1:

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py \
  mode:=real \
  model:=m1509 \
  host:=192.168.137.100 \
  rt_host:=192.168.137.100
```

Terminal 2:

```bash
ros2 run dsr_example pose_to_moveit_bridge --ros-args \
  -p action_name:=/move_action \
  -p pose_topic:=/my_topic/simulink/target_pose \
  -p group_name:=manipulator \
  -p link_name:=link_6 \
  -p frame_id:=base_link \
  -p orientation_enabled:=false
```

Terminal 3:

- start the Simulink block
- publish one target pose

## Troubleshooting

### `MoveIt result code: 1`

Success. The robot planned and executed the target.

### `MoveIt result code: -4`

Planning failed.

Common reasons:

- the target is unreachable
- the target is too close to a collision object
- the target frame is wrong
- the quaternion is invalid
- orientation constraints are too strict

### `tf2_echo` says the frame does not exist at first

This is often a startup timing issue. Wait for the robot state publisher and TF tree to settle, then try again.

### Real robot planning works but execution fails

Check the real-robot notes carefully, especially:

- `rt_host` must match the controller IP
- the teach pendant must be active
- the controller must be in the correct state

## Relevant Files

- `dsr_example/simple/pose_to_moveit_bridge.py`
- `dsr_example/simple/single_robot_simple.py`
- `setup.py`
- `package.xml`

## References

- Doosan Jazzy repository: https://github.com/DoosanRobotics/doosan-robot2/tree/jazzy
- ROS 2 Jazzy installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Doosan MoveIt tutorial: https://doosanrobotics.github.io/doosan-robotics-ros-manual/jazzy/tutorials/basic_tutorials/moveit_tutorial.html
