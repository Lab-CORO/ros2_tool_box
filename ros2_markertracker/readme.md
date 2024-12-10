# ROS2 Marker Tracker

## Build
```
colcon build --packages-select ros2_markertracker_interfaces ros2_markertracker
source install/setup.bash
```

## Launch markertracker node
```
ros2 run ros2_markertracker markertracker_node
```
### Parameters
- `input_image_topic`: ros2 topic to get the 2d image
- `publish_topic_image_result`: Publish a rviz marker (bool)
- `camera_info_topic`: ros2 topic camera info (the node wait 10s to get the camera info, you should turn on the camera driver before)
- `marker_length`: the aruco tag size (unite: cm)
- `aruco_dictionary_id`: the arcu dict (ex: "DICT_4X4_250")
- `camera_frame_id`: The frame used for calibration
- `ignore_marker_ids_array` : (Currently useless)

### Published topic
- `/image_result`
- `/tf`: tf name marker
- `/poses`: rviz marke (arrow)
- `/fiducial_markers`: marker pose with covariance