from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_a = Node(
            namespace='service_roboter',
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_a',
            parameters=[
                        {'marker_size': 0.1},
                        {'aruco_dictionary_id': 'DICT_6X6_250'},
                        {'camera_frame' : 'camera_a'},
            ],
            remappings=[
                        ("/camera/camera_info", "/service_roboter/camera/a/camera_info"),
                        ("/camera/image_raw"  , "/service_roboter/camera/a/image_raw"),
                        ("/camera/image_out"  , "/service_roboter/camera/a/image_out"),
                        ("/service_roboter/aruco_poses", "/service_roboter/aruco/a/poses"),
                        ("/service_roboter/aruco_markers", "/service_roboter/aruco/a/markers")
            ],
    )

    aruco_b = Node(
            namespace='service_roboter',
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_b',
            parameters=[
                        {'marker_size': 0.1},
                        {'aruco_dictionary_id': 'DICT_6X6_250'},
                        {'camera_frame' : 'camera_b'},
            ],
            remappings=[
                        ("/camera/camera_info", "/service_roboter/camera/b/camera_info"),
                        ("/camera/image_raw"  , "/service_roboter/camera/b/image_raw"),
                        ("/camera/image_out"  , "/service_roboter/camera/b/image_out"),
                        ("/service_roboter/aruco_poses", "/service_roboter/aruco/b/poses"),
                        ("/service_roboter/aruco_markers", "/service_roboter/aruco/b/markers")
            ],
    )

    aruco_frame_a = Node(
            namespace='service_roboter',
            package='adt_cv',
            executable='aruco_frame',
            name='aruco_frame_a',
            parameters=[
                        {'markers_topic': '/service_roboter/aruco/a/markers'},
                        {'marker_id': 0},
                        {'parent_frame': 'aruco_0'},
                        {'child_frame': 'camera_a'},
                        {'inverse_frame': True},
            ],
    )
    
    aruco_frame_b = Node(
            namespace='service_roboter',
            package='adt_cv',
            executable='aruco_frame',
            name='aruco_frame_b',
            parameters=[
                        {'markers_topic': '/service_roboter/aruco/b/markers'},
                        {'marker_id': 0},
                        {'parent_frame': 'aruco_0'},
                        {'child_frame': 'camera_b'},
                        {'inverse_frame': True},
            ],
    )

    drink_a = Node(
            namespace='service_roboter',
            package='adt_cv',
            executable='drink_detection',
            name='drink_detection_a',
            parameters=[
                        {'image_topic': '/service_roboter/camera/a/image_raw'},
                        {'camera_info_topic': '/service_roboter/camera/a/camera_info'},
                        {'parent_frame': 'camera_a'},
                        {'marker_frame': 'aruco_0'},
                        {'marker_id': 0},
                        {'marker_topic': '/service_roboter/aruco/a/markers'},
                        {'pixel_per_meter': 1181},
            ],
            remappings=[
                        ("/service_roboter/image_out", "/service_roboter/drink/a/image_out"),
                        ("/service_roboter/foxglove_out", "/service_roboter/drink/a/foxglove_out"),
                        ("/service_roboter/poses", "/service_roboter/drink/a/poses")
            ],
    )

    drink_b = Node(
            namespace='service_roboter',
            package='adt_cv',
            executable='drink_detection',
            name='drink_detection_b',
            parameters=[
                        {'image_topic': '/service_roboter/camera/b/image_raw'},
                        {'camera_info_topic': '/service_roboter/camera/b/camera_info'},
                        {'parent_frame': 'camera_b'},
                        {'marker_frame': 'aruco_0'},
                        {'marker_id': 0},
                        {'marker_topic': '/service_roboter/aruco/b/markers'},
                        {'pixel_per_meter': 1181},
            ],
            remappings=[
                        ("/service_roboter/image_out", "/service_roboter/drink/b/image_out"),
                        ("/service_roboter/foxglove_out", "/service_roboter/drink/b/foxglove_out"),
                        ("/service_roboter/poses", "/service_roboter/drink/b/poses")
            ],
    )

    return LaunchDescription([
        aruco_a,
        aruco_b,
        aruco_frame_a,
        aruco_frame_b,
        drink_a,
        drink_b,
        ])