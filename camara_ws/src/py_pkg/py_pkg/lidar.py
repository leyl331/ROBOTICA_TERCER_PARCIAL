import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge
import numpy as np

class DepthToLaserScan(Node):

    def __init__(self):
        super().__init__('depth_to_scan')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/kinect/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/kinect/depth/camera_info', self.camera_info_callback, 10)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.fx = None  # Focal length (x) de la cámara
        self.cx = None  # Centro óptico x
        self.frame_id = "camera_depth_frame"

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.get_logger().info("📷 Camera info recibida")

    def depth_callback(self, msg):
        if self.fx is None:
            return  # Esperar camera_info

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            height, width = depth_image.shape

            center_row = depth_image[height // 2, :]  # Fila central
            scan_msg = LaserScan()

            scan_msg.header.stamp = msg.header.stamp
            scan_msg.header.frame_id = self.frame_id
            scan_msg.angle_min = -0.5  # aproximadamente +/- 30°
            scan_msg.angle_max = 0.5
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / width
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1.0 / 30.0
            scan_msg.range_min = 0.3
            scan_msg.range_max = 5.0

            ranges = []

            for i in range(width):
                depth = center_row[i] / 1000.0  # de mm a metros
                if depth == 0.0 or np.isnan(depth):
                    ranges.append(float('inf'))
                else:
                    ranges.append(depth)

            scan_msg.ranges = ranges
            self.scan_pub.publish(scan_msg)

        except Exception as e:
            self.get_logger().error(f"Error procesando profundidad: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserScan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
