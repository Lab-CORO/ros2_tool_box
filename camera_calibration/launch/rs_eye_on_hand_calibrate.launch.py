from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Déclaration des arguments de lancement

    namespace_prefix_arg = DeclareLaunchArgument("namespace_prefix", default_value="rs_calib")
    marker_id_arg = DeclareLaunchArgument("marker_id", default_value="1")
    marker_size_arg = DeclareLaunchArgument("marker_size", default_value="0.096")
    eye_on_hand_arg = DeclareLaunchArgument("eye_on_hand", default_value="true")
    marker_frame_arg = DeclareLaunchArgument("marker_frame", default_value="aruco_marker")
    ref_frame_arg = DeclareLaunchArgument("ref_frame", default_value="base_link")
    corner_refinement_arg = DeclareLaunchArgument("corner_refinement", default_value="LINES")
    camera_frame_arg = DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame")
    camera_image_topic_arg = DeclareLaunchArgument("camera_image_topic", default_value="/camera/camera/color/image_rect_raw")
    camera_info_topic_arg = DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info")



    # Lancer le driver ROS de realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
        )
    )

    # Chemin des fichiers de lancement pour d'autres packages
    dsr_bringup2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("dsr_bringup2"), "launch", "dsr_bringup2_rviz.launch.py")
        ),
        launch_arguments={
            "mode": "real",
            "host": "192.168.137.100",
            "port": "12345",
            "model": "m1013"
        }.items()
    )



    # Configuration de l’Aruco
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="ros2_aruco",
        parameters=[
            {"image_is_rectified": True},
            {"marker_size": LaunchConfiguration("marker_size")},
            {"aruco_dictionary_id": "DICT_4X4_250"},
            {"marker_id": LaunchConfiguration("marker_id")},
            {"reference_frame": LaunchConfiguration("ref_frame")},
            {"camera_frame": LaunchConfiguration("camera_frame")},
            {"marker_frame": LaunchConfiguration("marker_frame")},
            {"corner_refinement": LaunchConfiguration("corner_refinement")},
            {"camera_info_topic": LaunchConfiguration("camera_info_topic")},
            {"image_topic": LaunchConfiguration("camera_image_topic")}
        ]
    )

    pose_array_to_tf_node = Node(
        package='camera_calibration',
        executable='pose_array_to_tf',
        name='pose_array_to_tf',
        parameters=[
            {'subscription_pose_array': 'aruco_poses'},
            {'parent_frame_id': LaunchConfiguration("camera_frame")},
            {'publisher_frame_id': LaunchConfiguration("marker_frame")}
        ]
    )

    handeye_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hand_eye_calibration'), 'calibration.launch.py'
            )
        ),
        launch_arguments={
            'calibration_type': 'eye-in-hand',
            'namespace_prefix': LaunchConfiguration('namespace_prefix'),
            'freehand_robot_movement': 'true',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'link_6',
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'aruco_marker'
        }.items()
    )

    return LaunchDescription([
        namespace_prefix_arg,
        marker_id_arg,
        marker_size_arg,
        eye_on_hand_arg,
        marker_frame_arg,
        ref_frame_arg,
        corner_refinement_arg,
        camera_frame_arg,
        camera_image_topic_arg,
        camera_info_topic_arg,
        dsr_bringup2_launch,
        realsense_launch,
        aruco_node,
        pose_array_to_tf_node,
        handeye_calibration_launch
    ])
