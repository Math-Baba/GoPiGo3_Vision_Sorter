import rclpy
from rclpy.node import Node
import time
import easygopigo3

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.get_logger().info("Motion node started")

        # Initialisation du robot
        self.gpg = easygopigo3.EasyGoPiGo3()

        # Lancer le test de mouvement
        self.move_robot()

    def move_robot(self):
        self.get_logger().info("Avance...")
        self.gpg.forward()
        time.sleep(2)

        self.gpg.stop()
        time.sleep(1)

        self.get_logger().info("Tourne 90°...")
        self.gpg.turn_degrees(90)

        self.gpg.stop()
        self.get_logger().info("Stop")

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()