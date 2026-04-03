import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json

class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        # HSV calibre
        self.colors = {
            'green': {'low': [35, 50, 50], 'high': [85, 255, 255]},
            'blue':  {'low': [90, 50, 50], 'high': [130, 255, 255]}
        }
        self.min_area = 800
        self.kernel_open = np.ones((5, 5), np.uint8)
        self.kernel_close = np.ones((15, 15), np.uint8)

        self.sub = self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(String, '/cube_detections', 10)
        self.get_logger().info('Cube detector pret - ecoute /image_raw')

    def detect_color(self, enhanced, color_name):
        cfg = self.colors[color_name]
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(cfg['low']), np.array(cfg['high']))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for c in contours:
            area = cv2.contourArea(c)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                detections.append({
                    'color': color_name,
                    'x': cx, 'y': cy,
                    'w': w, 'h': h,
                    'area': int(area)
                })
        return detections

    def image_cb(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)

        all_detections = []
        for color_name in self.colors:
            all_detections += self.detect_color(enhanced, color_name)

        # Trier par aire (plus gros = plus proche)
        all_detections.sort(key=lambda d: d['area'], reverse=True)

        out = String()
        out.data = json.dumps({
            'detections': all_detections,
            'frame_width': msg.width,
            'frame_height': msg.height
        })
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
