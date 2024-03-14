from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    relay_driver = Node(
            namespace='service_roboter',
            package='adt_code',
            executable='basis_roboter_relay_driver',
            name='basis_roboter_relay_driver',
            parameters=[{'serial_port':'/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.1.4:1.0'}],
    )

    motor_driver_a = Node(
            namespace='service_roboter',
            package='adt_code',
            executable='basis_roboter_motor_driver',
            name='basis_roboter_motor_driver_a',
            parameters=[{'serial_port':'/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.1.2:1.0-port0'},
                        {'x_axis_scale': 1.0},
                        {'y_axis_scale': 1.0},
                        {'z_axis_scale': 1.0}],
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
            parameters=[{'serial_port':'/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.1.3:1.0-port0'},
                        {'x_axis_scale': 1.0},
                        {'y_axis_scale':-1.0},
                        {'z_axis_scale': 1.0}],
            remappings=[
            ("/service_roboter/command", "/service_roboter/motor/b/command"),
            ("/service_roboter/command_twist", "/service_roboter/motor/b/command_twist"),
            ],
    )


    return LaunchDescription([relay_driver, motor_driver_a, motor_driver_b])