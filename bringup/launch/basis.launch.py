from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    relay_driver = Node(
            namespace='service_roboter',
            package='adt_code',
            executable='basis_roboter_relay_driver',
            name='basis_roboter_relay_driver',
            parameters=[{'serial_port':'/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_85036313130351C0C0C0-if00'}],
    )

    motor_driver_a = Node(
            namespace='service_roboter',
            package='adt_code',
            executable='basis_roboter_motor_driver',
            name='basis_roboter_motor_driver_a',
            parameters=[{'serial_port':'/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.2:1.0-port0'},
                        {'x_axis_scale': 1.0},
                        {'y_axis_scale': 1.0},
                        {'z_axis_scale': -1.0}],
            remappings=[
            ("/service_roboter/command", "/service_roboter/motor/a/command"),
            ("/service_roboter/command_twist", "/service_roboter/motor/a/command_twist"),
            ],
    )

    motor_driver_b = Node(
            namespace='service_roboter',
            package='adt_code',
            executable='basis_roboter_motor_driver',
            name='basis_roboter_motor_driver_b',
            parameters=[{'serial_port':'/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.3:1.0-port0'},
                        {'x_axis_scale': -1.0},
                        {'y_axis_scale': 1.0},
                        {'z_axis_scale': -1.0}],
            remappings=[
            ("/service_roboter/command", "/service_roboter/motor/b/command"),
            ("/service_roboter/command_twist", "/service_roboter/motor/b/command_twist"),
            ],
    )

    motor_manager = Node(
            namespace='service_roboter',
            package='adt_code',
            executable='basis_roboter_motor_manager',
            name='basis_roboter_motor_manager',
            parameters=[
                {'motor_driver_a_twist_topic': '/service_roboter/motor/a/command_twist'},
                {'motor_driver_b_twist_topic': '/service_roboter/motor/b/command_twist'},
                {'motor_manager_twist_topic': '/service_roboter/motor/manager/command_twist'},
                ],
    )

    camera_a = Node(
            namespace='service_roboter',
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_a',
            parameters=[{'video_device':'/dev/video0'},
                        {'frame_id':'camera_a'},
                        {'framerate'   : 2.0},
                        {'image_width' : 2048},
                        {'image_height': 1536},
                        {'camera_name' : 'camera_a'},
                        {'pixel_format': 'yuyv'},
                        # {'av_device_format':'MJPEG'},
                        {'camera_info_url': 'file:///home/adt/ros_ws/bringup/config/camera_calibration_a.yaml'},
                        {'brightness': 0},
            ],
            remappings=[
                        ("/service_roboter/camera_info", "/service_roboter/camera/a/camera_info"),
                        ("/service_roboter/image_raw"  , "/service_roboter/camera/a/image_raw"),
                        ("/service_roboter/image_raw/compressed", "/service_roboter/camera/a/image_raw/compressed"),
                        ("/service_roboter/image_raw/compressedDepth", "/service_roboter/camera/a/image_raw/compressedDepth"),
                        ("/service_roboter/image_raw/theora", "/service_roboter/camera/a/image_raw/theora"),
            ],
    )

    camera_b = Node(
            namespace='service_roboter',
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_b',
            parameters=[{'video_device':'/dev/video2'},
                        {'frame_id':'camera_b'},
                        {'framerate'   : 2.0},
                        {'image_width' : 2048},
                        {'image_height': 1536},
                        {'camera_name' : 'camera_b'},
                        {'pixel_format': 'yuyv'},
                        # {'av_device_format':'MJPEG'},
                        {'camera_info_url': 'file:///home/adt/ros_ws/bringup/config/camera_calibration_b.yaml'},
                        {'brightness': 0},
            ],
            remappings=[
                        ("/service_roboter/camera_info", "/service_roboter/camera/b/camera_info"),
                        ("/service_roboter/image_raw"  , "/service_roboter/camera/b/image_raw"),
                        ("/service_roboter/image_raw/compressed", "/service_roboter/camera/b/image_raw/compressed"),
                        ("/service_roboter/image_raw/compressedDepth", "/service_roboter/camera/b/image_raw/compressedDepth"),
                        ("/service_roboter/image_raw/theora", "/service_roboter/camera/b/image_raw/theora"),
            ],
    )

    return LaunchDescription([
        relay_driver,
        motor_driver_a, 
        motor_driver_b, 
        motor_manager,
        camera_a,
        camera_b,
        ])