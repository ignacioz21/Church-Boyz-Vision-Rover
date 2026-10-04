import re

with open("central_brain.py", "r") as f:
    content = f.read()

# Add bind to the UDP broadcast socket so it uses the WiFi interface
content = re.sub(r'self\.udp_sock\.setsockopt\(socket\.SOL_SOCKET, socket\.SO_BROADCAST, 1\)',
                 '''self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            self.udp_sock.bind(("192.168.88.11", 0))
        except:
            pass''', content)

with open("central_brain.py", "w") as f:
    f.write(content)

with open("web_brain.py", "r") as f:
    content = f.read()

# Also bind web_brain's direct UDP socket if it creates one, wait, web_brain shares brain.udp_sock!
