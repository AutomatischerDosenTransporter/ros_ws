import math
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from foxglove_msgs.msg import SceneEntity, SceneUpdate 
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers


class DrinkNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("drink_node")

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="parent_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Origin of image frame",
            ),
        )

        self.declare_parameter(
            name="marker_id",
            value=0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Id of marker on witch the drinks get projected",
            ),
        )

        self.declare_parameter(
            name="marker_topic",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Frame on witch the lines get projected",
            ),
        )

        self.declare_parameter(
            name="marker_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Frame of the marker",
            ),
        )

        self.declare_parameter(
            name="pixel_per_meter",
            value=1000,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Pixel pro Meter",
            ),
        )

        image_topic = (self.get_parameter("image_topic").get_parameter_value().string_value)
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (self.get_parameter("camera_info_topic").get_parameter_value().string_value)
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.parent_frame = (self.get_parameter("parent_frame").get_parameter_value().string_value)
        self.get_logger().info(f"Parent frame: {self.parent_frame}")

        self.marker_id = (self.get_parameter("marker_id").get_parameter_value().integer_value)
        self.get_logger().info(f"Marker id: {self.marker_id}")

        self.marker_topic = (self.get_parameter("marker_topic").get_parameter_value().string_value)
        self.get_logger().info(f"Market topic: {self.marker_topic}")
        
        self.marker_frame = (self.get_parameter("marker_frame").get_parameter_value().string_value)
        self.get_logger().info(f"Market frame: {self.marker_frame}")
        
        self.pixel_per_meter = (self.get_parameter("pixel_per_meter").get_parameter_value().integer_value)
        self.get_logger().info(f"Pixel per Meter: {self.pixel_per_meter}")
        
        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.marker_sub = self.create_subscription(ArucoMarkers, self.marker_topic, self.marker_callback, qos_profile_sensor_data)

        # Set up publishers
        self.image_pub = self.create_publisher(Image, "image_out", 10)
        self.foxglove_pub = self.create_publisher(SceneUpdate, "foxglove_out", 10)
        self.pose_pub = self.create_publisher(PoseArray, "poses", 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.marker = None

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

    def marker_callback(self, markers):
        for i in range(0, len(markers.marker_ids)):
            if markers.marker_ids[i] == self.marker_id:
                self.marker = markers.poses_2d[i]
                return

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
        
        if self.marker is None:
            self.get_logger().warn("No markers has been received!")
            return

        # self.get_logger().info(f"Plane frame: {str(self.info_msg.header)}")
        # try:
        #     plane_frame = self.tf_buffer.lookup_transform(self.parent_frame, self.plane_frame, self.info_msg.header.stamp)
        # except:
        #     self.get_logger().warn("Mssing Frames!")
        #     return
        
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'mono8')
        face_cascade = cv2.CascadeClassifier('/home/adt/ros_ws/bringup/config/drink_cascade.xml')
        faces = face_cascade.detectMultiScale(cv_image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        cloud = PoseArray()
        cloud.header.frame_id = self.marker_frame
        cloud.header.stamp = self.info_msg.header.stamp

        scene = SceneEntity()
        scene.id = self.parent_frame+"_foxglove"
        scene.frame_id = self.parent_frame
        scene.timestamp = self.info_msg.header.stamp


        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

            image_hight = cv_image.shape[0]
            image_width = cv_image.shape[1] 

            drink_x = (x - self.marker.x) / self.pixel_per_meter
            drink_y = (y - self.marker.y) / self.pixel_per_meter

            point = Pose()
            point.position.x = float(drink_x)        
            point.position.y = float(drink_y)          
            point.position.z = float(0)
            cloud.poses.append(point)          


        scene_update = SceneUpdate()
        scene_update.entities.append(scene)
        self.foxglove_pub.publish(scene_update)

        img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'mono8')
        img_msg.header.frame_id = self.parent_frame
        self.image_pub.publish(img_msg)

        self.pose_pub.publish(cloud)


def main():
    rclpy.init()
    node = DrinkNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
