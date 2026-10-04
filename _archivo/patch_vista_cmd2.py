import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Create a global command queue
    if "cmd_queue = []" not in content:
        content = content.replace("import cv2", "import cv2\ncmd_queue = []")
        
    old_get = """        import urllib.parse
        parsed_path = urllib.parse.urlparse(self.path)
        if parsed_path.path == '/command':
            qs = urllib.parse.parse_qs(parsed_path.query)
            cmd = qs.get('cmd', [''])[0].lower()
            if cmd in ['ready', 'stop', 'abort', 'quit']:
                # Comunicar el comando al arbitro principal
                if hasattr(self.server, 'arbitro') and self.server.arbitro is not None:
                    res = self.server.arbitro.intentar(cmd)
                    print(f"[VISTA-WEB] Comando: {cmd} -> {res}")
            self.send_response(200)"""
            
    new_get = """        import urllib.parse
        parsed_path = urllib.parse.urlparse(self.path)
        if parsed_path.path == '/command':
            qs = urllib.parse.parse_qs(parsed_path.query)
            cmd = qs.get('cmd', [''])[0].lower()
            if cmd in ['ready', 'stop', 'abort', 'quit']:
                global cmd_queue
                cmd_queue.append(cmd)
                print(f"[VISTA-WEB] Encolado comando: {cmd}")
            self.send_response(200)"""
            
    content = content.replace(old_get, new_get)
    
    # Now patch tecla()
    old_tecla = """    def tecla(self) -> str:
        \"\"\"Lee el teclado de la ventana OpenCV.\"\"\"
        if not self._abierta:
            return ""
        k = cv2.waitKey(1) & 0xFF"""
        
    new_tecla = """    def tecla(self) -> str:
        \"\"\"Lee el teclado de la ventana OpenCV y web.\"\"\"
        global cmd_queue
        if cmd_queue:
            return cmd_queue.pop(0)
        if not self._abierta:
            return ""
        k = cv2.waitKey(1) & 0xFF"""
        
    content = content.replace(old_tecla, new_tecla)

    with open(filepath, 'w') as f:
        f.write(content)

patch("vision-system/vision/vista.py")
print("Vision HTTP server updated to use cmd_queue")
