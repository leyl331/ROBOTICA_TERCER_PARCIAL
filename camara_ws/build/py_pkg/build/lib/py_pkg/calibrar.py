

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
import numpy as np

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

        # Cargar la calibración de la cámara desde el archivo YAML
        self.load_calibration()

    def load_calibration(self):
        try:
            with open('/home/leyla/Desktop/fotos/camera_calibration/calibration.yaml', 'r') as f:
                loaded_dict = yaml.load(f, Loader=yaml.FullLoader)
            self.mtx = np.array(loaded_dict['camera_matrix'])
            self.dist = np.array(loaded_dict['dist_coeff'])
        except Exception as e:
            self.get_logger().error(f"Error al cargar la calibración: {e}")
            self.mtx, self.dist = None, None

    def listener_callback(self, msg):
        try:
            # Convertir imagen de ROS a OpenCV
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Verificar si la calibración está disponible
            if self.mtx is not None and self.dist is not None:
                # Obtener las dimensiones de la imagen
                h, w = image.shape[:2]
                # Obtener la nueva matriz de la cámara y la región de interés
                new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))

                # Corregir la distorsión en la imagen
                undistorted_image = cv2.undistort(image, self.mtx, self.dist, None, new_camera_mtx)

                # Convertir la imagen a escala de grises
                gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

                # Mostrar las imágenes corregidas
                cv2.imshow("Imagen Original (Corregida)", undistorted_image)
                cv2.imshow("Imagen en Gris (Corregida)", gray)
            else:
                # Si la calibración no está disponible, mostrar las imágenes sin corrección
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
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
