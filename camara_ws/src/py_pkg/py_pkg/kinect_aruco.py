import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import cv2
import numpy as np
import yaml

class KinectArucoDetector(Node):
    def __init__(self):
        super().__init__('kinect_aruco_detector')
        self.bridge = CvBridge()

        # Suscribirse a la imagen RGB de Kinect
        self.sub = self.create_subscription(
            Image,
            '/kinect/image_raw',
            self.image_callback,
            10
        )

        # Publicar centroide (opcional)
        self.pub = self.create_publisher(Point, '/marker_pose', 10)

        # Cargar calibración
        try:
            with open('/home/leyla/Desktop/fotos/camera_calibration/calibration.yaml', 'r') as f:
                data = yaml.safe_load(f)
                self.camera_matrix = np.array(data['camera_matrix'])
                self.dist_coeffs = np.array(data['dist_coeff'])
                self.get_logger().info("✅ Calibración cargada")
        except Exception as e:
            self.get_logger().error(f" Error al cargar calibración: {e}")
            self.camera_matrix = np.eye(3)
            self.dist_coeffs = np.zeros((5, 1))

        # ArUco
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, 0.1, self.camera_matrix, self.dist_coeffs) # m a cm

                for i in range(len(ids)):
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

                    # Centroide
                    c = corners[i][0]
                    cx = int(np.mean(c[:, 0]))
                    cy = int(np.mean(c[:, 1]))
                    z = tvecs[i][0][2] * 100

                    # Texto y círculo
                    text = f"ID:{ids[i][0]} Distancia : {z:.2f}cm"
                    cv2.putText(frame, text, (cx - 50, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 4)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), 1)

                    # Publicar punto
                    point = Point()
                    point.x = float(tvecs[i][0][0])
                    point.y = float(tvecs[i][0][1])
                    point.z = float(tvecs[i][0][2])
                    self.pub.publish(point)

            # Mostrar imagen
            cv2.imshow("Kinect ArUco View", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f" Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KinectArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
