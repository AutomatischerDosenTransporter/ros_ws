import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState


class MyNode(Node):

    def __init__(self):
        super().__init__('basis_roboter_demo')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        joint_state_position = JointState()
        joint_state_position.name = ["head_x_joint", "head_y_joint","head_z_joint"]
        joint_state_position.position = [0.5, 0.5, 0.5]
        self.publisher_.publish(joint_state_position)


def main(args=None):
    rclpy.init(args=args)

    myNode = MyNode()
    rclpy.spin(myNode)
    myNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()