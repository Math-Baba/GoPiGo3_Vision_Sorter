import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()

        # Ouvre la caméra
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # Encode l'image en JPEG
                _, jpeg = cv2.imencode('.jpg', frame)

                # Écrit dans le flux HTTP
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n')
        except Exception as e:
            print(f"Erreur stream: {e}")
        finally:
            cap.release()

if __name__ == "__main__":
    print("Stream sur http://0.0.0.0:8080")
    HTTPServer(('0.0.0.0', 8080), StreamHandler).serve_forever()