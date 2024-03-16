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
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import Point as GeometryPoint
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from Geometry3D import Line, Vector, Point
from foxglove_msgs.msg import SceneEntity, LinePrimitive, SceneUpdate 


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
            name="plane_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Frame on witch the lines get projected",
            ),
        )


        image_topic = (self.get_parameter("image_topic").get_parameter_value().string_value)
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (self.get_parameter("camera_info_topic").get_parameter_value().string_value)
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.parent_frame = (self.get_parameter("parent_frame").get_parameter_value().string_value)
        self.get_logger().info(f"Parent frame: {self.parent_frame}")

        self.plane_frame = (self.get_parameter("plane_frame").get_parameter_value().string_value)
        self.get_logger().info(f"Plane frame: {self.plane_frame}")
        
        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)

        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "drink_poses", 10)
        self.image_pub = self.create_publisher(Image, "image_out", 10)
        self.foxglove_pub = self.create_publisher(SceneUpdate, "foxglove_out", 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # self.get_logger().info(f"Plane frame: {str(self.info_msg.header)}")
        # try:
        #     plane_frame = self.tf_buffer.lookup_transform(self.parent_frame, self.plane_frame, self.info_msg.header.stamp)
        # except:
        #     self.get_logger().warn("Mssing Frames!")
        #     return
        
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'mono8')
        face_cascade = cv2.CascadeClassifier('/home/adt/ros_ws/bringup/config/drink_cascade.xml')
        faces = face_cascade.detectMultiScale(cv_image, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        scene = SceneEntity()
        scene.id = self.parent_frame+"_foxglove_"+self.plane_frame
        scene.frame_id = self.parent_frame
        scene.timestamp = self.info_msg.header.stamp

        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            # self.get_logger().info("Face!")
            image_hight = cv_image.shape[0]
            image_width = cv_image.shape[1]       
            vield_of_view = 120    
            alpha = (((x+w/2)/image_width)-0.5) * vield_of_view * (math.pi/180)
            beta = (((y+h/2)/image_hight)-0.5) * vield_of_view * (math.pi/180)
            camera_origin = Point(0, 0, 0)

            line_dir_x=math.sin(alpha)
            line_dir_y=math.sin(beta)
            line_dir_z=math.cos(alpha)+math.cos(beta)

            # line_dir=Vector(line_dir_x, line_dir_y, line_dir_z)
            # line = Line(camera_origin, line_dir)

            primitiveLine = LinePrimitive()
            primitiveLine.pose.position.x = float(camera_origin.x)
            primitiveLine.pose.position.y = float(camera_origin.y)
            primitiveLine.pose.position.z = float(camera_origin.z)
            primitiveLine.color.r = 100.0
            primitiveLine.color.g = 100.0
            primitiveLine.color.b = 100.0
            primitiveLine.color.a = 100.0
            primitiveLine.thickness = 0.005

            geometry_point = GeometryPoint()
            geometry_point.x = camera_origin.x + line_dir_x * 5.0
            geometry_point.y = camera_origin.y + line_dir_y * 5.0
            geometry_point.z = camera_origin.z + line_dir_z * 5.0
            primitiveLine.points.append(GeometryPoint())
            primitiveLine.points.append(geometry_point)
            scene.lines.append(primitiveLine)




        pose_array = PoseArray()
        if self.parent_frame == "":
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            pose_array.header.frame_id = self.parent_frame


        

        pose_array.header.stamp = img_msg.header.stamp
        self.poses_pub.publish(pose_array)


        scene_update = SceneUpdate()
        scene_update.entities.append(scene)
        self.foxglove_pub.publish(scene_update)

        img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'mono8')
        img_msg.header.frame_id = self.parent_frame
        self.image_pub.publish(img_msg)


def main():
    rclpy.init()
    node = DrinkNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
