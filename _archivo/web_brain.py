import json
import socket
import math
import time
import threading
import central_brain
from central_brain import CentralBrain

from flask import Flask, send_from_directory
from flask_socketio import SocketIO

app = Flask(__name__, static_folder='dashboard')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
# central_brain crea su propio SocketIO al importarse, pero ese nunca se sirve:
# sus web_log (TX) se perdían. Se reemplaza por el que sí atiende al dashboard.
central_brain.socketio = socketio

@app.route('/')
def index():
    return send_from_directory('dashboard', 'index.html')

@socketio.on('ui_command')
def handle_command(data):
    cmd = data.get('cmd')
    if cmd == 'IDLE':
        brain.mode = "IDLE"
        brain.assignments = {10: -1, 11: -1}
        brain.states = {10: "IDLE", 11: "IDLE"}
        # Send abort instantly via UDP
        for r in [10, 11]:
            brain.udp_sock.sendto(f"{r}:r".encode(), ("192.168.88.255", 8889))
        web_log("BRAIN", "MODO CAMBIADO A: IDLE (Abort Broadcast)")
    elif cmd == 'PLAN_10':
        brain.mode = "PLAN_10"
        web_log("BRAIN", "MODO CAMBIADO A: PLAN_10")
    elif cmd == 'EXEC_10':
        brain.mode = "EXEC_10"
        web_log("BRAIN", "MODO CAMBIADO A: EXEC_10")
    elif cmd == 'PLAN_11':
        brain.mode = "PLAN_11"
        web_log("BRAIN", "MODO CAMBIADO A: PLAN_11")
    elif cmd == 'EXEC_11':
        brain.mode = "EXEC_11"
        web_log("BRAIN", "MODO CAMBIADO A: EXEC_11")
    elif cmd == 'PLAN_ALL':
        brain.mode = "PLAN_ALL"
        web_log("BRAIN", "MODO CAMBIADO A: PLAN_ALL")
    elif cmd == 'AUTO':
        brain.mode = "AUTO"
        web_log("BRAIN", "MODO CAMBIADO A: AUTO")

def web_log(tag, msg):
    socketio.emit('log', {'source': 'BRAIN', 'tag': tag, 'msg': msg})

# Patch print_map to emit map state to WebSockets
def patched_print_map(self, msg, routes):
    if time.time() - getattr(self, 'last_web_map', 0) > 0.1: # 10Hz
        self.last_web_map = time.time()
        
        # Format routes for frontend
        rt = {}
        for rid in [10, 11]:
            if routes.get(rid):
                r = routes[rid]
                rt[str(rid)] = {
                    "p1": [r["p1"][0], r["p1"][1]],
                    "p2": [r["p2"][0], r["p2"][1]],
                    "p3": [r["p3"][0], r["p3"][1]]
                }
                
        socketio.emit('map_state', {
            'grid': msg.get("grid", {"cols": 800, "rows": 600}),
            'rovers': msg.get("rovers", []),
            'cubes': msg.get("cubes", []),
            'depots': msg.get("depots", []),
            'routes': rt
        })
    
    # Optional: Still call original print_map if desired, or skip it.
    pass 

# Patch UDP sendto to also log to web
original_sendto = None
def patched_sendto(data, addr):
    msg = data.decode()
    web_log("TX", msg)
    return original_sendto(data, addr)

if __name__ == '__main__':
    global brain
    brain = CentralBrain()
    
    brain.print_map = patched_print_map.__get__(brain)
    
    original_sendto = brain.udp_sock.sendto
    # brain.udp_sock.sendto = patched_sendto
    
    # Start brain in background thread
    brain_thread = threading.Thread(target=brain.run, daemon=True)
    brain_thread.start()
    
    print("[WEB] Servidor web iniciado en http://0.0.0.0:8891")
    socketio.run(app, host='0.0.0.0', port=8891, debug=False, use_reloader=False)
