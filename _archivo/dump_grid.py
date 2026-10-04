import socket, json
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 8888))
sock.settimeout(2.0)
try:
    data, _ = sock.recvfrom(4096)
    msg = json.loads(data.decode('utf-8'))
    print("GRID:", msg.get("grid"))
    print("CUBES:", [(c.get("color"), c.get("col"), c.get("row")) for c in msg.get("cubes", [])])
except Exception as e:
    print(e)
