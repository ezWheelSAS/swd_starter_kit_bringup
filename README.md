# SWD® Starter Kit ROS Stack

This package contains a sample ROS configuration for the SWD® Starter Kit.

We've tried to make the main launch file is
[`starter_kit.launch`](launch/starter_kit.launch) configurable, the file accepts
a set of arguments to which, we set the default values as used in the SWD
Starter Kit.

## Devices' arguments

- `master_ip` (default `"10.0.0.1"`): The ethernet IP address of the host machine
  (the robot) used to connect with the LiDAR.
- `lidar_sensor` (default `"idec_se2l"`): The LiDAR sensor name, it can be one
  of [`idec_se2l`, `sick_nanoscan3` or `pilz_psen`]. The package contains
  configuration files for this three LiDARs, however, if you are using another
  model, you can easily add the corresponding launch file.
- `lidar_sensor_ip` (default `"10.0.0.5"`): The IP address of the LiDAR, used by
  the LiDAR driver.
- `scan_prefix` (default `"laser_1"`): A prefix which will be used for LiDAR
  topics.
- `joy_config` (default `"xbox"`): A config parameter used with the `joy/joy_node`,
  used only when `enable_joystick==true`.

## Global arguments

- `scan_topic` (default `"[scan_prefix]/scan"`): The output LiDAR topic.
- `scan_frame_id` (default `"[scan_prefix]"`): The laser scan frame ID.
- `odom_frame_id` (default `"odom"`): The odometry frame ID.
- `base_frame_id` (default `"base_link"`): The moving base frame ID.
- `global_frame_id` (default `"map"`): The global map frame ID.
- `baseline_m` (default `"0.485"`): The distance between the two wheels, this value is used
  with `swd_ros_controllers/swd_diff_drive_controller` to calculate the odometry.
- `enable_websocket` (default `true`): Use the `rosbridge_server/rosbridge_websocket` and
  `tf2_web_republisher/tf2_web_republisher` to route ROS messages to the web UI.
- `enable_joystick` (default `true`): Enable joystick support, uses `joy/joy_node` and
  `teleop_twist_joy/teleop_node`.

## Control/filtering arguments

- `enable_cmd_vel_mux` (default `"true"`): A boolean parameter to enable the
  multiplexer on the controller's `cmd_vel` input.
- `enable_laser_filter` (default `"true"`): A boolean parameter to enable the
  `laser_filters/scan_to_scan_filter_chain` configuration.

## Navigation mode arguments

- `nav_mode` (default `"mapping"`): The navigation mode in which the robot will
  be launched, currently, two mode are preconfigured: `mapping` and
  `localization`. In the `mapping` mode, a SLAM algorithm is used to build a map
  of the environment, this map can then be saved as a static map (using
  `map_server/map_saver`). Then, we can launch the robot in the localization
  mode to localize itself in the map. If you do not need to launch a localization
  or mapping stack, set `nav_mode` is set to `none`.
- `nav_algo` (default `"hector"`): The algorithm to be used for the selected
  mode, it accepts [`hector`, `lama` or `gmapping`] when `nav_mode=='mapping'`,
  and [`amcl` or `lama`] when `nav_mode=='localization'`, note that for
  `'localization'` mode, you will need to provide an initial pose (via RViz, or
  by publishing a `geometry_msgs/PoseWithCovariance` message to `/initial_pose`),
  or if using `amcl`, you can trigger a global localization strategy
  by calling the `rosservice call /global_localization {}`, notes that
  currently, LAMA's `loc2d_ros` does not support global localization.
- `nav_localization_static_map` (default `"assets/map_box2_v2.yaml"`): The
  static map to be used with the localization algorithm, this map should be
  obtained from the mapping process and saved using `map_server/map_saver`. This
  package comes with a sample map, which you can visualize for testing purposes.

# Dependencies

The package contains configurations for the following nodes:

- `swd_ros_controllers/swd_diff_drive_controller` is used to send commands to
  the motors and publish the odometry and the safety functions flags.
- `swd_robot_manager/robot_manager` is used to implement some high level operations.
- `swd_robot_manager/pose_from_tf` is used to build a
  `geometry_msgs/PoseStamped` message from the TF tree (technically, applies the
  localization/SLAM `odom->map` drift correction to the odometry's `base_link->odom`).
- `rosbridge_server/rosbridge_websocket` and `tf2_web_republisher/tf2_web_republisher` 
   when `enable_websocket==true`.
- `joy/joy_node` and `teleop_twist_joy/teleop_node` are used when `enable_joystick==true`, 
  enables the usage of an USB joystick. 
- `topic_tools/mux` is used when `enable_cmd_vel_mux==true` to implement a
  multiplexer (named `mux_cmd_vel`) in the ROS graph, the output is a
  `geometry_msgs/Twist` message linked the differential drive velocity control topic
  `swd_diff_drive_controller/cmd_vel`. The node takes multiple topics as input,
  the output selection can be done by calling
  `rosrun topic_tools mux_select mux_cmd_vel <INPUT_TOPIC>`, where
  `<INPUT_TOPIC>` is one of:
  - `joy_cmd_vel` to receive commands from the joystick (selected by default),
  - `key_cmd_vel` to receive commands from the keyboard (for example, via
    `teleop_twist_keyboard/teleop_twist_keyboard.py`),
  - `remote_cmd_vel` to receive commands from a remote node (ROS-Mobile mobile
    app for example),
  - `nav_cmd_vel` to receive commands from a navigation stack (`move_base` stuff
    for example).
- `urg_node/urg_node` when `lidar_sensor=='idec_se2l'`.
- `psen_scan_v2/psen_scan_v2_node` when `lidar_sensor=='pilz_psen'`.
- `sick_safetyscanners/sick_safetyscanners_node` when `lidar_sensor=='sick_nanoscan3'`.
- `laser_filters/scan_to_scan_filter_chain` is used when
  `enable_laser_filter==true` to filter the LiDAR measurements, the current
  configuration set ranges to be in $-\pi/2$ to $\pi/2$.
- `hector_mapping/hector_mapping` when `nav_mode=='mapping'` and `nav_algo=='hector'`.
- `gmapping/slam_gmapping` when `nav_mode=='mapping'` and `nav_algo=='gmapping'`.
- `iris_lama_ros/slam2d_ros` when `nav_mode=='mapping'` and `nav_algo=='lama'`.
- `iris_lama_ros/loc2d_ros` when `nav_mode=='localization'` and `nav_algo=='lama'`.
- `amcl/amcl` when `nav_mode=='localization'` and `nav_algo=='amcl'`.
- `tf/static_transform_publisher` is used to publish static TFs for the LiDAR sensors.

# Usage with the `ezw-ros-bringup` Systemd service

The SWD® Starter Kit comes with a set of *Systemd* services which launches the
Linux and ROS stacks automatically at startup.

By default, the `ezw-ros-bringup` service launches `swd_starter_kit_bringup/starter_kit.launch` 
with the default arguments, however, if you want to modify the arguments, you can
create the special configuration file `~/.swd_ros_env` and set the
`SWD_BRINGUP_LAUNCH_FILE_ARGS` to the desired list of arguments, for example, 
if we want to launch the `starter_kit.launch` with arguments `nav_mode:=localization`, 
`nav_algo:=amcl`, `enable_laser_filter:=false` and `enable_cmd_vel_mux:=false`, we can add
the following line to `~/.swd_ros_env`:

``` shell
SWD_BRINGUP_LAUNCH_FILE_ARGS=("nav_mode:=localization" "nav_algo:=amcl" "enable_laser_filter:=false" "enable_cmd_vel_mux:=false")
```

Other options can be set in the `~/.swd_ros_env`, by default, the service will
load the workspace found in the user's home directory `~/ros_ws`. However, you
can choose a custom ROS workspace to be loaded at startup by adding this to your
`~/.swd_ros_env` file:

``` shell
SWD_USER_WS_DIR="$HOME/robot_workspace/"
```

The service will check if the chosen workspace exists, and sources:

1. `SWD_USER_WS_DIR/install/setup.bash` if it exists.
2. `SWD_USER_WS_DIR/devel/setup.bash` if it exists.

If you want to change the package and the launch file to be loaded at startup,
with the arguments to send to the `roslaunch`. You can set these options:

``` shell
# The package from which you want to load the startup/bringup
# launch file
SWD_BRINGUP_PKG="<YOUR_BRINGUP_PACKAGE>"

# The launch file to be loaded with `roslaunch`
SWD_BRINGUP_LAUNCH_FILE="<YOUR_LAUNCH_FILE>.launch"

# The list of arguments (as "arg:=value" ...)
SWD_BRINGUP_LAUNCH_FILE_ARGS=("arg1:=value1" "arg2:=value2" ...)
```

ez-Wheel® 2021
