import rclpy
from rclpy.node import Node
from robot_interfaces.msg import CubeDetection
import easygopigo3

DEAD_ZONE    = 40
SEARCH_SPEED = 150
ALIGN_SPEED  = 100
FORWARD_SPEED = 200

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.gpg = easygopigo3.EasyGoPiGo3()
        self.subscription = self.create_subscription(
            CubeDetection,
            'cube_detection',
            self.on_detection,
            10
        )
        self.get_logger().info('Motion node démarré')

    def on_detection(self, msg):
        if not msg.detected or not msg.color:
            self.get_logger().info('Cube non détecté → rotation recherche')
            self.gpg.set_speed(SEARCH_SPEED)
            self.gpg.turn_degrees(90, blocking=False)
            return

        frame_cx = msg.frame_width // 2
        offset = msg.cx - frame_cx

        if abs(offset) <= DEAD_ZONE:
            self.get_logger().info(f'Cube {msg.color} centré → avance')
            self.gpg.set_speed(FORWARD_SPEED)
            self.gpg.forward()
        elif offset < 0:
            self.get_logger().info(f'Cube à gauche (offset {offset}) → correction gauche')
            self.gpg.set_speed(ALIGN_SPEED)
            self.gpg.left()
        else:
            self.get_logger().info(f'Cube à droite (offset {offset}) → correction droite')
            self.gpg.set_speed(ALIGN_SPEED)
            self.gpg.right()

def main():
    rclpy.init()
    node = MotionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()