import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String


class BaseDriveSerialBridge(Node):

    def __init__(self):
        super().__init__('base_drive_serial_bridge')
        ser = serial.Serial('COM1')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    base_drive_serial_bridge = BaseDriveSerialBridge()
    rclpy.spin(base_drive_serial_bridge)
    base_drive_serial_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
