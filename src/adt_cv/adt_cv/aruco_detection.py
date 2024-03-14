import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.marker_size = 0.1 # Size of the ArUco marker in meters
        
        fx = fy = 500  # Focal length in pixels (assumed)
        cx = cy = 320  # Principal point (image center) in pixels (assumed)
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

        k1 = k2 = p1 = p2 = k3 = 0.0  # Assuming no distortion
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Define the dictionary of ArUco markers
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        # Initialize the detector parameters using default values
        parameters = aruco.DetectorParameters()

        # Detect markers in the image
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if len(corners) > 0:
            for i in range(0, len(ids)):
       
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.025, self.camera_matrix, self.dist_coeffs)
                
                cv2.aruco.drawDetectedMarkers(cv_image, corners) 

                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.01) 
     





        # Display the resulting frame
        cv2.imshow('Aruco Detection', cv_image)
        cv2.waitKey(1)


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