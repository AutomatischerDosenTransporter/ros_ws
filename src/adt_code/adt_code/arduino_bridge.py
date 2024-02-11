import rclpy
import serial
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        
        self.declare_parameter('cmd_vel', 'cmd_vel')
        par_cmd_vel = self.get_parameter('cmd_vel').get_parameter_value().string_value

        self.subscription = self.create_subscription(Twist, par_cmd_vel, self.listener_callback, 10)
        self.subscription  # prevent unused variable warning


        
        
        self.declare_parameter('serial_port_a', '/dev/ttyUSB0')
        par_serial_port_a = self.get_parameter('serial_port_a').get_parameter_value().string_value

        self.declare_parameter('serial_port_b', '/dev/ttyUSB1')
        par_serial_port_b = self.get_parameter('serial_port_b').get_parameter_value().string_value

        global serialX
        serialX = serial.Serial(par_serial_port_a, 9600, timeout=1)
        global serialY
        serialY = serial.Serial(par_serial_port_b, 9600, timeout=1)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        # serialX.write(b'speed -axis x -speed 0')

        




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()