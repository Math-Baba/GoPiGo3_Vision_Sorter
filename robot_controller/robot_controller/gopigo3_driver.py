import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import easygopigo3 as easy

class GoPiGo3Driver(Node):
    def __init__(self):
        super().__init__('gopigo3_driver')
        self.gpg = easy.EasyGoPiGo3()
        self.gpg.set_speed(0)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.get_logger().info('GoPiGo3 driver pret - ecoute /cmd_vel')

    def cmd_vel_cb(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_base = 0.117
        max_speed = 300
        scale = 300

        left = linear - (angular * wheel_base / 2)
        right = linear + (angular * wheel_base / 2)

        left_speed = int(-left * scale)
        right_speed = int(-right * scale)

        left_speed = max(-max_speed, min(max_speed, left_speed))
        right_speed = max(-max_speed, min(max_speed, right_speed))

        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, right_speed)
        self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, left_speed)

    def destroy_node(self):
        self.gpg.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GoPiGo3Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
