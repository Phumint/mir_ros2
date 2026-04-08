# MiR100 Bridge for ROS2

The main purpose of this repo is the bridge script that gets and inject ROS2 data from and into the MiR100.  

The driver is made for the purpose of the MiR being integrated as a mobile base for a mobile manipulator hybrid (see [here](https://github.com/Phumint/mobile_manipulator_ROS2)).

> **Note:** Aside from the MiR100, support for other types of MiR has not been tested with (though, it should still work).

---

## Acknowledgments

This project and its code builds upon and is heavily inspired by the phenomenal open-source work done by the DFKI-NI team on the original ROS 1 `mir_robot` repository. 

Specifically, this ROS 2 port utilizes their robust URDF setups from the `mir_description` package, and our `mir_gazebo` simulation approach draws direct inspiration from their implementations. 

You can find their original repository (Noetic branch) here: 
[DFKI-NI/mir_robot (noetic)](https://github.com/DFKI-NI/mir_robot/tree/noetic)

I would also like to acknowledge the use of the `dual_laser_merger` package created by pradyum, which is integrated into this project. You can find that repository here:
[pradyum/dual_laser_merger](https://github.com/pradyum/dual_laser_merger.git)

Additionally, this project was developed with coding assistance from Google Gemini. All AI-generated code was manually reviewed, tested, and modified by me to ensure it met project requirements and integrated correctly with the ROS 2 architecture.

## Requirements
- ROS 2 (Humble/Iron/Jazzy supported)
- Gazebo (Ignition, NOT Classic)

## Cloning the Packages

```bash
# Navigate to the your workspace's src folder, in my case it is "mir_ws" 
cd ~/mir_ws/src

# Clone this repository
git clone https://github.com/Phumint/mir_ros2.git

# Build all the packages
cd ~/mir_ws
colcon build --symlink-install 

# Source your workspace
source install/setup.bash
```

## Launching the MiR Driver Bridge (Real Hardware)
> **Note:** For optimal speed and stability, I highly recommended to connect to the MiR 100 via a wired Ethernet connection rather than Wi-Fi.

Run the following command to launch the driver. Replace the IP address with your robot's specific IP (the launch file's default is `192.168.12.20`):

```bash
ros2 launch mir_driver_bridge mir.launch.py mir_hostname:=10.38.11.17
```
Verify the connection by listing the active topics:
```bash
ros2 topic list
```
(*Expected output should include: `/odom`, `/b_scan`, `/f_scan`, `/scan`, `/tf`, and `/tf_static`*)

If you want to launch with RViz2:
```bash
ros2 launch mir_driver_bridge mir.launch.py mir_hostname:=10.38.11.17 rviz:=true
```

## Launching the MiR in Gazebo

Run the following command to launch the MiR in a Gazebo Simulation of an empty world:

```bash
ros2 launch mir_gazebo mir_empty_world.launch.py
```
If you want to launch with RViz2:
```bash
ros2 launch mir_gazebo mir_empty_world.launch.py rviz:=true
```

| <img src="https://github.com/user-attachments/assets/b629ee84-331a-4cc7-b19b-ecc370ad54ba" alt="RViz2 View" width="450" height="300" /> | <img src="https://github.com/user-attachments/assets/3de32e14-157a-4c44-adae-cbd0e76f04de" alt="Gazebo View" width="450" height="300" /> |
| :---: | :---: |
| *Robot state and sensor data visualized in RViz2* | *MiR 100 spawned in an empty Gazebo world* |

## MiR100 ROS Data

<table border="0" cellspacing="0" cellpadding="4" style="border: none;">
  <thead>
    <tr>
      <th colspan="3" style="text-align: left; border: none;">Node List</th>
    </tr>
  </thead>
  <tbody>
    <tr><td>/LightCtrl</td><td>/camera_floor/transform2</td><td>/mir_parameter_persister</td></tr>
    <tr><td>/MC</td><td>/diagnostic_aggregator</td><td>/mir_pose_pub</td></tr>
    <tr><td>/MissionController</td><td>/laser_back/driver</td><td>/mir_precision_docking</td></tr>
    <tr><td>/PCDiagnostics</td><td>/laser_back/footpr_filter_relay</td><td>/mir_session_importer_module</td></tr>
    <tr><td>/SickPLC</td><td>/laser_back/relay</td><td>/mir_velocity_regulator</td></tr>
    <tr><td>/StateTF</td><td>/laser_back/transform</td><td>/mircontrol</td></tr>
    <tr><td>/ailogtool_logger</td><td>/laser_front/driver</td><td>/mirspawn</td></tr>
    <tr><td>/api_4855_1775546788637</td><td>/laser_front/footpr_filter_relay</td><td>/move_base_node</td></tr>
    <tr><td>/battery_node</td><td>/laser_front/relay</td><td>/resource_tracker</td></tr>
    <tr><td>/battery_voltage_speed_limiter</td><td>/laser_front/transform</td><td>/robot_tracker</td></tr>
    <tr><td>/camera_floor/camera_floor_base_link2</td><td>/map_server</td><td>/rosapi</td></tr>
    <tr><td>/camera_floor/camera_floor_base_link3</td><td>/marker_tracking_node</td><td>/rosbridge_websocket</td></tr>
    <tr><td>/camera_floor/driver</td><td>/metrics_manager</td><td>/rosout</td></tr>
    <tr><td>/camera_floor/transform</td><td>/mirEventTrigger</td><td>/scan_filter</td></tr>
    <tr><td>/mirSound</td><td>/mir_amcl</td><td>/supervisor</td></tr>
    <tr><td>/mir_auto_bagger</td><td>/mir_autologger</td><td>/transform_footprint</td></tr>
    <tr><td>/mir_external_if</td><td>/mir_ip_link_diagnostics_node</td><td>/transform_imu</td></tr>
    <tr><td>/mir_mailer</td><td>/mir_modbus</td><td>/web_map_generator</td></tr>
    <tr><td>/mir_param_manager</td><td>/wifi_diagnostics</td><td>/wifi_watchdog</td></tr>
  </tbody>
</table>

<br>

<table border="0" cellspacing="0" cellpadding="4" style="border: none;">
  <thead>
    <tr>
      <th colspan="3" style="text-align: left; border: none;">Topic List</th>
    </tr>
  </thead>
  <tbody>
    <tr><td>/LightCtrl/bms_data</td><td>/data_events/maps</td><td>/move_base_node/current_goal</td></tr>
    <tr><td>/LightCtrl/charging_state</td><td>/data_events/mission_groups</td><td>/move_base_node/global_costmap/inflated_obstacles</td></tr>
    <tr><td>/LightCtrl/us_list</td><td>/data_events/positions</td><td>/move_base_node/global_costmap/obstacles</td></tr>
    <tr><td>/MC/battery_currents</td><td>/data_events/registers</td><td>/move_base_node/global_costmap/parameter_descriptions</td></tr>
    <tr><td>/MC/battery_percentage</td><td>/data_events/sessions</td><td>/move_base_node/global_costmap/parameter_updates</td></tr>
    <tr><td>/MC/battery_voltage</td><td>/data_events/sounds</td><td>/move_base_node/global_costmap/robot_footprint</td></tr>
    <tr><td>/MC/currents</td><td>/data_events/user_groups</td><td>/move_base_node/global_costmap/unknown_space</td></tr>
    <tr><td>/MC/encoders</td><td>/data_events/users</td><td>/move_base_node/global_plan</td></tr>
    <tr><td>/MissionController/CheckArea/visualization_marker</td><td>/diagnostics</td><td>/move_base_node/local_costmap/inflated_obstacles</td></tr>
    <tr><td>/MissionController/prompt_user</td><td>/diagnostics_agg</td><td>/move_base_node/local_costmap/obstacles</td></tr>
    <tr><td>/PB/gpio/input</td><td>/diagnostics_toplevel_state</td><td>/move_base_node/local_costmap/parameter_descriptions</td></tr>
    <tr><td>/PB/gpio/stop_button</td><td>/f_raw_scan</td><td>/move_base_node/local_costmap/parameter_updates</td></tr>
    <tr><td>/SickPLC/parameter_descriptions</td><td>/f_scan</td><td>/move_base_node/local_costmap/robot_footprint</td></tr>
    <tr><td>/SickPLC/parameter_updates</td><td>/f_scan_footpr_filter</td><td>/move_base_node/local_costmap/safety_zone</td></tr>
    <tr><td>/active_mapping_guid</td><td>/global_regulated_cmd_vel</td><td>/move_base_node/local_costmap/unknown_space</td></tr>
    <tr><td>/amcl_pose</td><td>/hook/data</td><td>/move_base_node/mir_escape_recovery/visualization_marker</td></tr>
    <tr><td>/b_raw_scan</td><td>/imu1_debug_data</td><td>/move_base_node/parameter_descriptions</td></tr>
    <tr><td>/b_scan</td><td>/imu2_debug_data</td><td>/move_base_node/parameter_updates</td></tr>
    <tr><td>/b_scan_footpr_filter</td><td>/imu_data</td><td>/move_base_node/time_to_coll</td></tr>
    <tr><td>/camera_floor/background</td><td>/initialpose</td><td>/move_base_node/traffic_costmap/inflated_obstacles</td></tr>
    <tr><td>/camera_floor/driver/color/camera_info</td><td>/joystick_vel</td><td>/move_base_node/traffic_costmap/obstacles</td></tr>
    <tr><td>/camera_floor/driver/color/image_raw</td><td>/laser_back/driver/parameter_descriptions</td><td>/move_base_node/traffic_costmap/parameter_descriptions</td></tr>
    <tr><td>/camera_floor/driver/color/image_raw/compressed</td><td>/laser_back/driver/parameter_updates</td><td>/move_base_node/traffic_costmap/parameter_updates</td></tr>
    <tr><td>/camera_floor/driver/color/image_raw/compressed/parameter_descriptions</td><td>/laser_front/driver/parameter_descriptions</td><td>/move_base_node/traffic_costmap/robot_footprint</td></tr>
    <tr><td>/camera_floor/driver/color/image_raw/compressed/parameter_updates</td><td>/laser_front/driver/parameter_updates</td><td>/move_base_node/traffic_costmap/unknown_space</td></tr>
    <tr><td>/camera_floor/driver/depth/camera_info</td><td>/light_cmd</td><td>/move_base_node/visualization_marker</td></tr>
    <tr><td>/camera_floor/driver/depth/color/points</td><td>/localization_score</td><td>/move_base_simple/goal</td></tr>
    <tr><td>/camera_floor/driver/depth/image_rect_raw</td><td>/map</td><td>/move_base_simple/visualization_marker</td></tr>
    <tr><td>/camera_floor/driver/depth/image_rect_raw/compressed</td><td>/map_metadata</td><td>/moving_state</td></tr>
    <tr><td>/camera_floor/driver/depth/image_rect_raw/compressed/parameter_descriptions</td><td>/ultrasonic_sensors/pointcloud_combined</td><td>/odom</td></tr>
    <tr><td>/camera_floor/driver/depth/image_rect_raw/compressed/parameter_updates</td><td>/wifi_diagnostics/cur_ap</td><td>/odom_enc</td></tr>
    <tr><td>/camera_floor/driver/detect_pallet_obstacles</td><td>/wifi_diagnostics/wifi_ap_interface_stats</td><td>/odom_imu1</td></tr>
    <tr><td>/camera_floor/driver/detect_rack_obstacles</td><td>/wifi_diagnostics/wifi_ap_time_stats</td><td>/odom_imu2</td></tr>
    <tr><td>/camera_floor/driver/extrinsics/depth_to_color</td><td>/marker_tracking_node/cancel</td><td>/one_way_map</td></tr>
    <tr><td>/camera_floor/driver/extrinsics/depth_to_infra1</td><td>/marker_tracking_node/feedback</td><td>/param_manager_update</td></tr>
    <tr><td>/camera_floor/driver/extrinsics/depth_to_infra2</td><td>/marker_tracking_node/goal</td><td>/param_update</td></tr>
    <tr><td>/camera_floor/driver/infra1/camera_info</td><td>/marker_tracking_node/laser_line_extract/parameter_descriptions</td><td>/particlevizmarker</td></tr>
    <tr><td>/camera_floor/driver/infra1/image_rect_raw</td><td>/marker_tracking_node/laser_line_extract/parameter_updates</td><td>/proximity/point_cloud</td></tr>
    <tr><td>/camera_floor/driver/infra1/image_rect_raw/compressed</td><td>/marker_tracking_node/laser_line_extract/visualization_marker</td><td>/resource_tracker/acquisition</td></tr>
    <tr><td>/camera_floor/driver/infra1/image_rect_raw/compressed/parameter_descriptions</td><td>/marker_tracking_node/result</td><td>/resource_tracker/needed_resources</td></tr>
    <tr><td>/camera_floor/driver/infra1/image_rect_raw/compressed/parameter_updates</td><td>/marker_tracking_node/status</td><td>/resource_tracker/world_model</td></tr>
    <tr><td>/camera_floor/driver/infra2/camera_info</td><td>/mirEventTrigger/events</td><td>/robot_mode</td></tr>
    <tr><td>/camera_floor/driver/infra2/image_rect_raw</td><td>/mir_amcl/selected_points</td><td>/robot_pose</td></tr>
    <tr><td>/camera_floor/driver/infra2/image_rect_raw/compressed</td><td>/mir_cmd</td><td>/robot_state</td></tr>
    <tr><td>/camera_floor/driver/infra2/image_rect_raw/compressed/parameter_descriptions</td><td>/mir_log</td><td>/robot_status</td></tr>
    <tr><td>/camera_floor/driver/infra2/image_rect_raw/compressed/parameter_updates</td><td>/mir_sound/sound_event</td><td>/robot_tracker/robots_pointcloud</td></tr>
    <tr><td>/camera_floor/driver/parameter_descriptions</td><td>/mir_status_msg</td><td>/robot_tracker/tracked_robots</td></tr>
    <tr><td>/camera_floor/driver/parameter_updates</td><td>/mirspawn/node_events</td><td>/rosout</td></tr>
    <tr><td>/camera_floor/driver/visualization_marker</td><td>/mirwebapp/grid_map_metadata</td><td>/rosout_agg</td></tr>
    <tr><td>/camera_floor/filter/visualization_marker</td><td>/mirwebapp/laser_map_metadata</td><td>/safety_status</td></tr>
    <tr><td>/camera_floor/floor</td><td>/mirwebapp/web_path</td><td>/scan</td></tr>
    <tr><td>/camera_floor/obstacles</td><td>/move_base/cancel</td><td>/scan_filter/visualization_marker</td></tr>
    <tr><td>/camera_floor_left/background</td><td>/move_base/feedback</td><td>/scan_footpr_filter</td></tr>
    <tr><td>/camera_floor_left/obstacles</td><td>/move_base/goal</td><td>/session_importer_node/info</td></tr>
    <tr><td>/camera_floor_right/background</td><td>/move_base/result</td><td>/set_mc_PID</td></tr>
    <tr><td>/camera_floor_right/obstacles</td><td>/move_base/status</td><td>/skid_detection_cusum</td></tr>
    <tr><td>/camera_top/background</td><td>/move_base_node/MIRPlannerROS/cost_cloud</td><td>/skid_detection_likelihood</td></tr>
    <tr><td>/camera_top/obstacles</td><td>/move_base_node/MIRPlannerROS/global_plan</td><td>/skid_detection_measurments</td></tr>
    <tr><td>/check_area/polygon</td><td>/move_base_node/MIRPlannerROS/len_to_goal</td><td>/stall_detection</td></tr>
    <tr><td>/check_pose_area/polygon</td><td>/move_base_node/MIRPlannerROS/local_plan</td><td>/tf</td></tr>
    <tr><td>/cmd_vel</td><td>/move_base_node/MIRPlannerROS/parameter_descriptions</td><td>/tf_static</td></tr>
    <tr><td>/data_events/area_events</td><td>/move_base_node/MIRPlannerROS/parameter_updates</td><td>/traffic_map</td></tr>
    <tr><td>/data_events/docking_offsets</td><td>/move_base_node/MIRPlannerROS/visualization_marker</td><td>/wifi_diagnostics</td></tr>
    <tr><td>/data_events/elevator_floors</td><td>/move_base_node/SBPLLatticePlanner/plan</td><td>/wifi_diagnostics/roam_events</td></tr>
    <tr><td>/data_events/elevators</td><td>/move_base_node/SBPLLatticePlanner/sbpl_lattice_planner_stats</td><td>/wifi_diagnostics/wifi_ap_rssi</td></tr>
    <tr><td>/data_events/footprints</td><td>/move_base_node/SBPLLatticePlanner/visualization_marker</td><td>/wifi_watchdog/ping</td></tr>
  </tbody>
</table>
