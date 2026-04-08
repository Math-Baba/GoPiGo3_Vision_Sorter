import cv2
import numpy as np
import json
import base64
import threading
import time
import os
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from robot_interfaces.msg import CubeDetection

COLOR_RANGES = {
    'green':  {'low': [35, 50, 50],   'high': [85, 255, 255],  'bgr': (0, 255, 0)},
    'yellow': {'low': [20, 100, 100], 'high': [35, 255, 255],  'bgr': (0, 255, 255)},
    'blue':   {'low': [90, 50, 50],   'high': [130, 255, 255], 'bgr': (255, 100, 0)},
    'red':    {'low': [0, 120, 70],   'high': [10, 255, 255],  'bgr': (0, 0, 255)},
}
MIN_AREA = 500

state = {
    'selected_color': None,
    'frame': None,
    'detected': False,
    'cx': 0,
    'frame_width': 320,
}
lock = threading.Lock()

WEB_DIR = os.path.join(
    get_package_share_directory('robot_controller'),
    'web'
)

def detect_color(frame, color_name):
    cfg = COLOR_RANGES[color_name]
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    if color_name == 'red':
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = cv2.inRange(hsv, np.array(cfg['low']), np.array(cfg['high']))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area > MIN_AREA:
            if best is None or area > cv2.contourArea(best):
                best = c

    if best is not None:
        x, y, w, h = cv2.boundingRect(best)
        cx = x + w // 2
        cv2.rectangle(frame, (x, y), (x+w, y+h), cfg['bgr'], 2)
        cv2.circle(frame, (cx, y + h // 2), 6, (0, 0, 255), -1)
        cv2.putText(frame, color_name.upper(), (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, cfg['bgr'], 2)
        fw = frame.shape[1]
        cv2.line(frame, (fw // 2, 0), (fw // 2, frame.shape[0]), (100, 100, 100), 1)
        return True, cx
    return False, 0

def camera_loop(node):
    cap = None
    for idx in [0, 10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 21, 22, 23, 31]:
        c = cv2.VideoCapture(idx)
        if c.isOpened():
            ret, _ = c.read()
            if ret:
                cap = c
                node.get_logger().info(f'Caméra trouvée sur /dev/video{idx}')
                break
            c.release()

    if cap is None:
        node.get_logger().error('Aucune caméra trouvée !')
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        with lock:
            color = state['selected_color']
            fw = frame.shape[1]
            state['frame_width'] = fw

        detected, cx = False, 0
        if color and color in COLOR_RANGES:
            detected, cx = detect_color(frame, color)

        with lock:
            state['detected'] = detected
            state['cx'] = cx
            _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            state['frame'] = base64.b64encode(jpg.tobytes()).decode('utf-8')

        msg = CubeDetection()
        msg.color = color or ''
        msg.detected = detected
        msg.cx = cx
        msg.frame_width = fw
        node.publisher.publish(msg)

        time.sleep(0.05)

    cap.release()

class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == '/':
            self._serve_file('index.html', 'text/html')
        elif path == '/style.css':
            self._serve_file('style.css', 'text/css')
        elif path == '/app.js':
            self._serve_file('app.js', 'application/javascript')
        elif path == '/frame':
            self._serve_frame()
        elif path == '/set_color':
            params = urllib.parse.parse_qs(parsed.query)
            if 'color' in params:
                with lock:
                    state['selected_color'] = params['color'][0]
            self._json({'ok': True})
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_file(self, filename, content_type):
        filepath = os.path.join(WEB_DIR, filename)
        try:
            with open(filepath, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-type', content_type)
            self.end_headers()
            self.wfile.write(content)
        except FileNotFoundError:
            self.send_response(404)
            self.end_headers()

    def _serve_frame(self):
        with lock:
            data = {
                'img': state['frame'] or '',
                'detected': state['detected'],
                'cx': state['cx'],
                'frame_width': state['frame_width'],
                'color': state['selected_color'] or '',
            }
        self._json(data)

    def _json(self, data):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def log_message(self, format, *args): pass

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(CubeDetection, 'cube_detection', 10)
        self.get_logger().info('Camera node démarré — http://11.255.255.201:8080')

def main():
    rclpy.init()
    node = CameraNode()

    threading.Thread(target=camera_loop, args=(node,), daemon=True).start()

    server = HTTPServer(('0.0.0.0', 8080), WebHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    
    print(WEB_DIR)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()