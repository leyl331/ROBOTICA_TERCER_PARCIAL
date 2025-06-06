
# ------------------------------
# VELOCIDAD
# ------------------------------
#sssssssssssss

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ComandoVelocidad(Node):
    def __init__(self):
        super().__init__('setpoint_vel')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.2  #  200ms 
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.08    # Velocidad hacia adelante (m/s)
        msg.angular.z = 0.1    # Giro suave (rad/s)
        self.pub.publish(msg)
        self.get_logger().info(f'/cmd_vel → linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    nodo = ComandoVelocidad()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
