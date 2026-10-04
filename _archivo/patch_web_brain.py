import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Imports
    if "from flask import" not in content:
        content = "import threading\nfrom flask import Flask, send_from_directory\nfrom flask_socketio import SocketIO\n" + content
    
    # 2. Setup Flask before CentralBrain class
    if "app = Flask" not in content:
        flask_setup = """
app = Flask(__name__, static_folder='dashboard')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

@app.route('/')
def index():
    return send_from_directory('dashboard', 'index.html')

def web_log(tag, msg):
    try:
        socketio.emit('log', {'source': 'BRAIN', 'tag': tag, 'msg': msg})
    except: pass

@socketio.on('ui_command')
def handle_command(data):
    if not 'brain_instance' in globals(): return
    cmd = data.get('cmd')
    if cmd == 'IDLE':
        brain_instance.mode = "IDLE"
        brain_instance.assignments = {10: -1, 11: -1}
        brain_instance.states = {10: "IDLE", 11: "IDLE"}
        for r in [10, 11]:
            brain_instance.udp_sock.sendto(f"{r}:r".encode(), ("255.255.255.255", 8889))
        web_log("BRAIN", "MODO CAMBIADO A: IDLE (Abort Broadcast)")
    else:
        brain_instance.mode = cmd
        web_log("BRAIN", f"MODO CAMBIADO A: {cmd}")

class CentralBrain:
"""
        content = content.replace("class CentralBrain:", flask_setup)
        
    # 3. Patch print_map to emit
    old_print_map = """    def print_map(self, msg, routes):
        if time.time() - self.last_map_print < 0.3:
            return
        self.last_map_print = time.time()"""
        
    new_print_map = """    def print_map(self, msg, routes):
        if time.time() - getattr(self, 'last_web_map', 0) > 0.1: # 10Hz
            self.last_web_map = time.time()
            rt = {}
            for rid in [10, 11]:
                if routes.get(rid):
                    r = routes[rid]
                    rt[str(rid)] = {"p1": [r["p1"][0], r["p1"][1]], "p2": [r["p2"][0], r["p2"][1]], "p3": [r["p3"][0], r["p3"][1]]}
            try:
                socketio.emit('map_state', {
                    'grid': msg.get("grid", {"cols": 800, "rows": 600}),
                    'rovers': msg.get("rovers", []),
                    'cubes': msg.get("cubes", []),
                    'depots': msg.get("depots", []),
                    'routes': rt
                })
            except: pass

        if time.time() - self.last_map_print < 0.3:
            return
        self.last_map_print = time.time()"""
    content = content.replace(old_print_map, new_print_map)
    
    # 4. Patch sendto to log
    old_send = "self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))"
    new_send = """self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                    try: web_log("TX", cmd.decode('utf-8'))
                    except: pass"""
    content = content.replace(old_send, new_send)
    
    old_send2 = "self.udp_sock.sendto(f\"{rid}:M,{ml:.2f},{mr:.2f}\".encode(), (ROVER_BCAST, ROVER_PORT))"
    new_send2 = """cmd = f"{rid}:M,{ml:.2f},{mr:.2f}".encode()
            self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))"""
    content = content.replace(old_send2, new_send2)

    # 5. Modify main block to start threads
    old_main = """if __name__ == "__main__":
    brain = CentralBrain()
    brain.run()"""
    
    new_main = """if __name__ == "__main__":
    global brain_instance
    brain_instance = CentralBrain()
    
    # Run the brain in a daemon thread
    t = threading.Thread(target=brain_instance.run, daemon=True)
    t.start()
    
    print("[WEB] Servidor Dashboard iniciado en http://0.0.0.0:8891")
    socketio.run(app, host='0.0.0.0', port=8891, debug=False, use_reloader=False)"""
    
    content = content.replace(old_main, new_main)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Central brain natively upgraded to Web Portal")
