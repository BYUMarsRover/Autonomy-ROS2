<!-- Created by Daniel Webb Oct 2024 -->

# rover_task_autonomy.launch - pkg: start
	rover_common.launch - pkg: start
		rover_drive.launch - pkg: mobility
			NODE: transition - pkg: mobility
		battery_info.launch - pkg: peripherals
			NODE: battery_info - pkg: peripherals
		rover_home_gui.launch - pkg: home_gui
			NODE: rover_dev_update - pkg: home_gui
			NODE: rover_camera_control - pkg: home_gui
		rover.launch - pkg: odometry
			NODE: f9p - pkg: ublox
			NODE: PositionVelocityTimeTranslator - pkg: odometry
		heartbeat_rover.launch - pkg: heartbeat
			NODE: heartbeat_rover - pkg: heartbeat
	auto.launch - pkg: autonomy
		zed2i.launch - pkg: zed_wrapper
			No source package zed_wrapper
		autonomy_camera.launch - pkg: usb_cam
			NODE: usb_cam - pkg: usb_cam
			NODE: image_view - pkg: image_view
		aruco_detect.launch - pkg: autonomy
			NODE: aruco_detect - pkg: aruco_detect
		NODE: fiducial_data - pkg: autonomy
		NODE: state_machine_node - pkg: autonomy
		NODE: buffer_node - pkg: autonomy
	autopilot_drive.launch - pkg: mobility
		NODE: wheel_manager - pkg: mobility
		NODE: drive_manager - pkg: mobility
		NODE: autopilot_manager - pkg: mobility
		NODE: aruco_autopilot_manager - pkg: mobility
		NODE: path_manager - pkg: mobility
	josh_estimation.launch - pkg: odometry
		NODE: ukf_se_odom - pkg: robot_localization
		NODE: ukf_se_map - pkg: robot_localization
		NODE: navsat_transform - pkg: robot_localization
		NODE: rover_state_singleton_creator - pkg: odometry
		NODE: imu_filter_madgwick - pkg: imu_filter_madgwick
		





