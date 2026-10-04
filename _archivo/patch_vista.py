import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # We need to inject the MJPEG server into Vista.
    # It's a class Vista.
    
    inject_imports = """
import cv2
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import socketserver

class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                frame = getattr(self.server, 'latest_frame', None)
                if frame is not None:
                    _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    try:
                        self.wfile.write(b'--frame\\r\\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(jpeg))
                        self.end_headers()
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(b'\\r\\n')
                    except Exception:
                        break
                time.sleep(0.05)
        else:
            self.send_response(404)
            self.end_headers()

class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    pass

global_httpd = None
"""
    
    # Add imports at the top
    if "class VideoStreamHandler" not in content:
        content = content.replace("import cv2", inject_imports)
        
        # Now hook into Vista.__init__
        hook_init = """        self._abierta = False
        
        global global_httpd
        if global_httpd is None:
            try:
                global_httpd = ThreadingHTTPServer(('0.0.0.0', 8080), VideoStreamHandler)
                t = threading.Thread(target=global_httpd.serve_forever, daemon=True)
                t.start()
                print("[VISTA] Servidor de Video MJPEG iniciado en puerto 8080")
            except Exception as e:
                print(f"[VISTA] Error iniciando servidor web: {e}")"""
                
        content = content.replace("        self._abierta = False", hook_init)
        
        # Now hook into cv2.imshow
        hook_imshow = """        cv2.imshow(self._titulo, lienzo)
        if global_httpd is not None:
            global_httpd.latest_frame = lienzo"""
            
        content = content.replace("        cv2.imshow(self._titulo, lienzo)", hook_imshow)

    with open(filepath, 'w') as f:
        f.write(content)

patch("vision-system/vision/vista.py")
print("MJPEG Streamer injected into Vision System")
