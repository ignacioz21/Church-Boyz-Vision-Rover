import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_get = """        if self.path == '/video_feed':
            self.send_response(200)"""
            
    new_get = """        import urllib.parse
        parsed_path = urllib.parse.urlparse(self.path)
        if parsed_path.path == '/command':
            qs = urllib.parse.parse_qs(parsed_path.query)
            cmd = qs.get('cmd', [''])[0].lower()
            if cmd in ['ready', 'stop', 'abort', 'quit']:
                # Comunicar el comando al arbitro principal
                if hasattr(self.server, 'arbitro') and self.server.arbitro is not None:
                    res = self.server.arbitro.intentar(cmd)
                    print(f"[VISTA-WEB] Comando: {cmd} -> {res}")
            self.send_response(200)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed_path.path == '/video_feed':
            self.send_response(200)"""

    content = content.replace(old_get, new_get)
    
    # We need to pass `arbitro` to the httpd server so it can execute commands.
    # We need to find where arbitro is created or passed.
    
    with open(filepath, 'w') as f:
        f.write(content)

patch("vision-system/vision/vista.py")
print("Vision HTTP server updated to accept commands")
