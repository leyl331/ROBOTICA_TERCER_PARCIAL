import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthImageSubscriber(Node):

    def __init__(self):
        super().__init__('depth_image_suscriber')


        self.subscription = self.create_subscription(
            Image,
            '/kinect/depth/image_raw',
            self.listener_callback,10)


        self.bridge = CvBridge()


    def listener_callback(self, msg):
        try:
            
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            cv2.imshow("Depth Image", depth_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DepthImageSubscriber()

    # Ejecutar el nodo ROS 2
    rclpy.spin(minimal_subscriber)

    # Cerrar las ventanas de OpenCV cuando se termine
    cv2.destroyAllWindows()
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
