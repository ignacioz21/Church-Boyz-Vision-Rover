import socket

s = socket.socket()
s.settimeout(2.0)
try:
    s.connect(("127.0.0.1", 2026))
    data = s.recv(4096)
    print("Recibido:", data.decode()[:200])
except Exception as e:
    print("Error:", e)
