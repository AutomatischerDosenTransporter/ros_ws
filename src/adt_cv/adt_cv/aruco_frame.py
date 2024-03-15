import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import numpy as np
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('aruco_frame')
        
        self.declare_parameter(
            name="marker_id",
            value=0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Marker to use.",
            ),
        )

        self.declare_parameter(
            name="markers_topic",
            value="/markers",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Marker topic to use.",
            ),
        )

        self.declare_parameter(
            name="parent_frame",
            value="aruco",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Parent Frame",
            ),
        )

        self.declare_parameter(
            name="child_frame",
            value="camera",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Child Frame",
            ),
        )

        self.declare_parameter(
            name="inverse_frame",
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Enable if you want to switch frames Frame",
            ),
        )

        self.markers_topic = (
            self.get_parameter("markers_topic").get_parameter_value().string_value
        )

        self.marker_id = (
            self.get_parameter("marker_id").get_parameter_value().integer_value
        )

        self.parent_frame = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )

        self.child_frame = (
            self.get_parameter("child_frame").get_parameter_value().string_value
        )

        self.inverse_frame = (
            self.get_parameter("inverse_frame").get_parameter_value().bool_value
        )

        self.aruco_sub = self.create_subscription(ArucoMarkers, self.markers_topic, self.aruco_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def aruco_callback(self, data):

        for i in range(0, len(data.marker_ids)):
            marker_id = data.marker_ids[i]
            pose = data.poses[i]

            if marker_id != self.marker_id:
                continue

            t = TransformStamped()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.child_frame
            
            if not self.inverse_frame:
                t.transform.translation.x = pose.position.x
                t.transform.translation.y = pose.position.y
                t.transform.translation.z = pose.position.z

                t.transform.rotation.x = pose.orientation.x
                t.transform.rotation.y = pose.orientation.y
                t.transform.rotation.z = pose.orientation.z
                t.transform.rotation.w = pose.orientation.w
            else:
                translation_from_b_to_a = Vector3(
                    x=-pose.position.x,
                    y=-pose.position.y,
                    z=-pose.position.z
                )

                # Calculate the rotation from B to A
                conjugate_rotation_from_a_to_b = pose.orientation
                conjugate_rotation_from_a_to_b.x *= -1
                conjugate_rotation_from_a_to_b.y *= -1
                conjugate_rotation_from_a_to_b.z *= -1

                # Rotate the translation from B to A by the conjugate of the rotation from A to B
                rotation_matrix = [
                    [1 - 2 * (conjugate_rotation_from_a_to_b.y**2 + conjugate_rotation_from_a_to_b.z**2),
                    2 * (conjugate_rotation_from_a_to_b.x * conjugate_rotation_from_a_to_b.y - conjugate_rotation_from_a_to_b.z * conjugate_rotation_from_a_to_b.w),
                    2 * (conjugate_rotation_from_a_to_b.x * conjugate_rotation_from_a_to_b.z + conjugate_rotation_from_a_to_b.y * conjugate_rotation_from_a_to_b.w)],
                    [2 * (conjugate_rotation_from_a_to_b.x * conjugate_rotation_from_a_to_b.y + conjugate_rotation_from_a_to_b.z * conjugate_rotation_from_a_to_b.w),
                    1 - 2 * (conjugate_rotation_from_a_to_b.x**2 + conjugate_rotation_from_a_to_b.z**2),
                    2 * (conjugate_rotation_from_a_to_b.y * conjugate_rotation_from_a_to_b.z - conjugate_rotation_from_a_to_b.x * conjugate_rotation_from_a_to_b.w)],
                    [2 * (conjugate_rotation_from_a_to_b.x * conjugate_rotation_from_a_to_b.z - conjugate_rotation_from_a_to_b.y * conjugate_rotation_from_a_to_b.w),
                    2 * (conjugate_rotation_from_a_to_b.y * conjugate_rotation_from_a_to_b.z + conjugate_rotation_from_a_to_b.x * conjugate_rotation_from_a_to_b.w),
                    1 - 2 * (conjugate_rotation_from_a_to_b.x**2 + conjugate_rotation_from_a_to_b.y**2)]
                ]

                # Multiply the rotation matrix by the translation from B to A
                new_translation_from_b_to_a = [
                    rotation_matrix[0][0] * translation_from_b_to_a.x + rotation_matrix[0][1] * translation_from_b_to_a.y + rotation_matrix[0][2] * translation_from_b_to_a.z,
                    rotation_matrix[1][0] * translation_from_b_to_a.x + rotation_matrix[1][1] * translation_from_b_to_a.y + rotation_matrix[1][2] * translation_from_b_to_a.z,
                    rotation_matrix[2][0] * translation_from_b_to_a.x + rotation_matrix[2][1] * translation_from_b_to_a.y + rotation_matrix[2][2] * translation_from_b_to_a.z
                ]

                t.transform.translation.x = new_translation_from_b_to_a[0]
                t.transform.translation.y = new_translation_from_b_to_a[1]
                t.transform.translation.z = new_translation_from_b_to_a[2]

                t.transform.rotation.x = pose.orientation.x
                t.transform.rotation.y = pose.orientation.y
                t.transform.rotation.z = pose.orientation.z
                t.transform.rotation.w = pose.orientation.w

                
                
            self.tf_broadcaster.sendTransform(t)
                


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()