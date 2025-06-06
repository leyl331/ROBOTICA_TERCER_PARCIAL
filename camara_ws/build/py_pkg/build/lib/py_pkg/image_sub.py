import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('opencv_node')


        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)


        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convertir imagen de ROS a OpenCV
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Convertir la imagen a escala de grises
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Mostrar las imágenes
            cv2.imshow("Imagen Original", image)
            cv2.imshow("Imagen en Gris", gray)
            
            # Esperar por una tecla y procesar la ventana
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error al procesar la imagen: {e}")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # Ejecutar el nodo ROS 2
    rclpy.spin(minimal_subscriber)

    # Cerrar las ventanas de OpenCV cuando se termine
    cv2.destroyAllWindows()
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
