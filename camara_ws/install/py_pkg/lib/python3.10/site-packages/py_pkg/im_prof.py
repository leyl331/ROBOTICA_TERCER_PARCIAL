import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# cv2.COLORMAP_TURBO
# cv2.COLORMAP_HOT
# cv2.COLORMAP_PARULA
# cv2.COLORMAP_MAGMA


class KinectRepublisher(Node):
    def __init__(self):
        super().__init__('kinect_republisher')
        self.bridge = CvBridge()

        # Suscripción a la imagen RGB
        self.rgb_sub = self.create_subscription(
            Image,
            '/kinect/image_raw',
            self.rgb_callback,
            10
        )

        # Suscripción a la imagen de profundidad
        self.depth_sub = self.create_subscription(
            Image,
            '/kinect/depth/image_raw',
            self.depth_callback,
            10
        )

        # Publicadores personalizados
        self.rgb_pub = self.create_publisher(Image, '/kinect_custom/image_rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/kinect_custom/image_depth', 10)

        self.get_logger().info("✅ Nodo KinectRepublisher activo")

    def rgb_callback(self, msg):
        try:
            # Convertir y mostrar la imagen RGB
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Kinect RGB", img)
            cv2.waitKey(1)

            # Publicar en nuevo tópico
            self.rgb_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error RGB: {e}")

    # def depth_callback(self, msg):
    #     try:
    #         # Convertir y mostrar la imagen de profundidad
    #         depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #         cv2.imshow("Kinect Depth", depth)
    #         cv2.waitKey(1)

    #         # Publicar en nuevo tópico
    #         self.depth_pub.publish(msg)
    #     except Exception as e:
    #         self.get_logger().error(f"Error Depth: {e}")


    def depth_callback(self, msg):
        try:
        # Convertir imagen de profundidad
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Normalizar a 0–255 (para visualización)
                depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                depth_normalized = depth_normalized.astype('uint8')

        # Aplicar colormap
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        # Mostrar
                cv2.imshow("Depth Colored", depth_colored)
                cv2.waitKey(1)

        # También puedes seguir publicando la imagen original si quieres
                self.depth_pub.publish(msg)

        except Exception as e:
                 self.get_logger().error(f"Error Depth: {e}")








def main(args=None):
    rclpy.init(args=args)
    node = KinectRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
