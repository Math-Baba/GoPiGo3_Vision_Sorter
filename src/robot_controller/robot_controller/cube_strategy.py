import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time
import math


class CubeStrategy(Node):
    """
    Strategie v5 - Navigation avec carte de l'arene.
    
    Etats:
    CHERCHER -> ALIGNER -> APPROCHER -> ORIENTER -> POUSSER -> RETOUR -> REORIENTER -> CHERCHER
    
    Nouveau: ORIENTER - le robot se tourne vers la zone de depot
    avant de pousser le cube.
    """

    def __init__(self):
        super().__init__('cube_strategy')

        self.state = 'CHERCHER'
        self.push_start = None
        self.push_color = None
        self.target_color = None
        self.confirm_count = 0
        self.confirm_color = None
        self.confirm_needed = 3
        self.last_detection_time = 0
        self.state_start_time = time.time()
        self.detections = []
        self.real_detections = []
        self.frame_width = 640

        # Blind spot
        self.last_cube_area = 0
        self.last_cube_color = None
        self.last_cube_time = 0

        # Robot state (from odom)
        self.robot_heading = 0.0  # radians
        self.robot_x = 0.0
        self.robot_y = 0.0

        # === PARAMETRES ===
        self.search_turn_speed = 0.4
        self.max_forward_speed = 0.3
        self.min_forward_speed = 0.15
        self.push_speed = 0.5     

        # PID
        self.kp = 0.003
        self.ki = 0.0001
        self.kd = 0.002
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_angular = 0.4

        # Seuils
        self.align_tolerance = 40
        self.close_area = 6000
        self.blind_spot_area = 5500
        self.push_duration = 5.0
        self.lost_timeout = 4.0
        self.state_timeout = 30.0

        # Direction de poussee (angle absolu)
        # 0 = direction initiale du robot (vers cote A: vert+bleu)
        # pi = direction opposee (vers cote B: jaune+rouge)
        self.push_angles = {
            'green': 0.0,
            'blue':  0.0,
            'yellow': math.pi,
            'red':   math.pi,
        }

        self.cubes_sorted = {}
        self.total_sorted = 0

        # ROS
        self.det_sub = self.create_subscription(String, '/cube_detections', self.detection_cb, 10)
        self.odom_sub = self.create_subscription(String, '/odom_simple', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info('=== CUBE SORTER v5 ===')
        self.get_logger().info('Navigation avec carte arene')
        self.get_logger().info('Cote A (devant): vert + bleu')
        self.get_logger().info('Cote B (derriere): jaune + rouge')
        self.get_logger().info('Etats: CHERCHER > ALIGNER > APPROCHER > ORIENTER > POUSSER > RETOUR > REORIENTER')

    def odom_cb(self, msg):
        data = json.loads(msg.data)
        self.robot_heading = math.radians(data['heading_deg'])
        self.robot_x = data['x']
        self.robot_y = data['y']

    def detection_cb(self, msg):
        data = json.loads(msg.data)
        self.detections = data.get('detections', [])
        self.frame_width = data.get('frame_width', 640)
        self.real_detections = [d for d in self.detections if not d.get('ghost', False)]

        if self.real_detections:
            self.last_detection_time = time.time()
            if self.target_color:
                target_dets = [d for d in self.real_detections if d['color'] == self.target_color]
                if target_dets:
                    best = target_dets[0]
                    self.last_cube_area = best['area']
                    self.last_cube_color = best['color']
                    self.last_cube_time = time.time()
            else:
                best = self.real_detections[0]
                self.last_cube_area = best['area']
                self.last_cube_color = best['color']
                self.last_cube_time = time.time()

    def cmd(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.pub.publish(t)

    def stop(self):
        self.cmd(0.0, 0.0)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'target': self.target_color,
            'sorted': self.cubes_sorted,
            'total': self.total_sorted,
            'last_area': self.last_cube_area,
            'heading': round(math.degrees(self.robot_heading), 1)
        })
        self.status_pub.publish(msg)

    def get_target_cube(self):
        if not self.real_detections:
            return None
        if self.target_color:
            targets = [d for d in self.real_detections if d['color'] == self.target_color]
            if targets:
                return targets[0]
            return None
        else:
            center = self.frame_width // 2
            return min(self.real_detections, key=lambda d: abs(d['x'] - center))

    def target_lost(self):
        if not self.target_color:
            return True
        target_dets = [d for d in self.real_detections if d['color'] == self.target_color]
        if target_dets:
            return False
        return time.time() - self.last_cube_time > self.lost_timeout

    def in_blind_spot(self):
        no_target = True
        if self.target_color:
            no_target = len([d for d in self.real_detections if d['color'] == self.target_color]) == 0
        else:
            no_target = len(self.real_detections) == 0
        was_close = self.last_cube_area > self.blind_spot_area
        recent = (time.time() - self.last_cube_time) < 2.0
        return no_target and was_close and recent

    def state_timed_out(self):
        return time.time() - self.state_start_time > self.state_timeout

    def change_state(self, new_state):
        old = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.integral = 0.0
        self.prev_error = 0.0
        self.get_logger().info(f'[{old}] --> [{new_state}] target={self.target_color}')
        self.publish_status()

    def pid_angular(self, error):
        self.integral += error
        self.integral = max(-1000, min(1000, self.integral))
        derivative = error - self.prev_error
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.max_angular, min(self.max_angular, output))

    def normalize_angle(self, angle):
        """Normalise un angle entre -pi et pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def loop(self):
        self.publish_status()

        if self.state == 'CHERCHER':
            self.state_chercher()
        elif self.state == 'ALIGNER':
            self.state_aligner()
        elif self.state == 'APPROCHER':
            self.state_approcher()
        elif self.state == 'ORIENTER':
            self.state_orienter()
        elif self.state == 'POUSSER':
            self.state_pousser()
        elif self.state == 'RETOUR':
            self.state_retour()
        elif self.state == 'REORIENTER':
            self.state_reorienter()

    # === ETATS ===

    def state_chercher(self):
        self.target_color = None
        cube = self.get_target_cube()
        if cube:
            if cube['color'] == self.confirm_color:
                self.confirm_count += 1
            else:
                self.confirm_color = cube['color']
                self.confirm_count = 1

            if self.confirm_count >= self.confirm_needed:
                self.stop()
                time.sleep(0.2)
                self.target_color = cube['color']
                self.confirm_count = 0
                self.confirm_color = None
                self.get_logger().info(f'[CHERCHER] {cube["color"]} CONFIRME (area={cube["area"]}) -> LOCK')
                self.change_state('ALIGNER')
                return
        else:
            self.confirm_count = 0
            self.confirm_color = None

        self.cmd(angular=self.search_turn_speed)

    def state_aligner(self):
        cube = self.get_target_cube()
        if not cube:
            if self.in_blind_spot():
                self.get_logger().info(f'[ALIGNER] Blind spot {self.target_color} -> POUSSER')
                self.push_start = time.time()
                self.push_color = self.target_color
                self.change_state('POUSSER')
                return
            if self.target_lost():
                self.stop()
                self.change_state('CHERCHER')
                return
            return

        if self.state_timed_out():
            self.stop()
            self.change_state('CHERCHER')
            return

        center_x = self.frame_width // 2
        error = cube['x'] - center_x

        if abs(error) < self.align_tolerance:
            self.stop()
            time.sleep(0.2)
            self.get_logger().info(f'[ALIGNER] {self.target_color} aligne! error={error}')
            self.change_state('APPROCHER')
            return

        angular = -self.pid_angular(error)
        self.cmd(angular=angular)

    def state_approcher(self):
        cube = self.get_target_cube()

        if not cube:
            if self.in_blind_spot():
                self.get_logger().info(f'[APPROCHER] Blind spot! -> POUSSER')
                self.push_start = time.time()
                self.push_color = self.target_color
                self.change_state('POUSSER')
                return
            if self.target_lost():
                self.stop()
                self.change_state('CHERCHER')
                return
            self.cmd(linear=self.min_forward_speed)
            return

        if self.state_timed_out():
            self.stop()
            self.change_state('CHERCHER')
            return

        center_x = self.frame_width // 2
        error = cube['x'] - center_x
        angular = -self.pid_angular(error)

        if cube['area'] > self.close_area:
            self.push_color = self.target_color
            self.push_start = time.time()
            self.push_color = self.target_color
            self.get_logger().info(f'[APPROCHER] {self.target_color} proche! -> POUSSER')
            self.change_state('POUSSER')
            return

        area_ratio = min(cube['area'] / self.close_area, 1.0)
        speed = self.max_forward_speed * (1.0 - area_ratio * 0.6)
        speed = max(self.min_forward_speed, speed)

        if abs(error) > 80:
            self.cmd(angular=angular)
            return

        self.get_logger().info(
            f'[APPROCHER] {self.target_color} area={cube["area"]} error={error} speed={speed:.2f}'
        )
        self.cmd(linear=speed, angular=angular * 0.7)

    def state_orienter(self):
        """Se tourner vers la zone de depot AVANT de pousser"""
        color = self.target_color or 'green'
        target_angle = self.push_angles.get(color, 0.0)

        # Difference entre heading actuel et direction cible
        angle_error = self.normalize_angle(target_angle - self.robot_heading)

        self.get_logger().info(
            f'[ORIENTER] {color} heading={math.degrees(self.robot_heading):.0f}° '
            f'target={math.degrees(target_angle):.0f}° error={math.degrees(angle_error):.0f}°'
        )

        # Tolerance de 15 degres
        if abs(angle_error) < math.radians(15):
            self.stop()
            time.sleep(0.2)
            self.push_start = time.time()
            self.push_color = self.target_color
            self.get_logger().info(f'[ORIENTER] Oriente vers zone {color}! -> POUSSER')
            self.change_state('POUSSER')
            return

        if self.state_timed_out():
            # Timeout: pousse quand meme
            self.push_start = time.time()
            self.push_color = self.target_color
            self.change_state('POUSSER')
            return

        # Tourner vers la direction cible
        turn_speed = 0.3 if abs(angle_error) > math.radians(30) else 0.15
        if angle_error > 0:
            self.cmd(angular=turn_speed)
        else:
            self.cmd(angular=-turn_speed)

    def state_pousser(self):
        elapsed = time.time() - self.push_start
        color = self.push_color or 'unknown'
        remaining = self.push_duration - elapsed

        if remaining <= 0:
            self.stop()
            self.cubes_sorted[color] = self.cubes_sorted.get(color, 0) + 1
            self.total_sorted += 1
            self.get_logger().info(
                f'[POUSSER] {color} livre! Total: {self.total_sorted} '
                f'({", ".join(f"{k}:{v}" for k, v in self.cubes_sorted.items())})'
            )
            self.change_state('RETOUR')
            return

        self.cmd(linear=self.push_speed)
        
    def state_retour(self):
        elapsed = time.time() - self.state_start_time
        if elapsed > 1.5:
            self.stop()
            time.sleep(0.2)
            self.change_state('REORIENTER')
            return
        self.cmd(linear=-self.max_forward_speed)

    def state_reorienter(self):
        """Tourner vers le centre de l'arene pour scanner"""
        # Tourner vers le centre (angle oppose a notre position)
        center_angle = math.atan2(-self.robot_y, -self.robot_x)
        angle_error = self.normalize_angle(center_angle - self.robot_heading)

        elapsed = time.time() - self.state_start_time

        if abs(angle_error) < math.radians(20) or elapsed > 3.0:
            self.stop()
            self.last_cube_area = 0
            self.last_cube_color = None
            self.target_color = None
            self.change_state('CHERCHER')
            return

        turn_speed = 0.4 if abs(angle_error) > math.radians(45) else 0.2
        if angle_error > 0:
            self.cmd(angular=turn_speed)
        else:
            self.cmd(angular=-turn_speed)


def main(args=None):
    rclpy.init(args=args)
    node = CubeStrategy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()