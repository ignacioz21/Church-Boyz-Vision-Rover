import subprocess
import os
import threading
from flask import Flask, send_from_directory
from flask_socketio import SocketIO
import socket
import json
import math
import time
import sys

VISION_HOST = "127.0.0.1"
VISION_PORT = 2026
ROVER_BCAST = "192.168.88.255"
ROVER_PORT = 8889

PRE_APPROACH_DIST = 8.5
SCALE = 2.0


app = Flask(__name__, static_folder='dashboard')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

@app.route('/')
def index():
    return send_from_directory('dashboard', 'index.html')

def web_log(tag, msg):
    # Emite directo al dashboard (mismo proceso); no bloquea el lazo del cerebro
    try:
        socketio.emit('log', {'source': 'BRAIN', 'tag': tag, 'msg': msg})
    except Exception:
        pass

@socketio.on('ui_command')
def handle_command(data):
    global brain_thread, brain_instance
    if not 'brain_instance' in globals(): return
    cmd = data.get('cmd')
    
    if cmd == 'RESTART_VISION':
        web_log("SYS", "Reiniciando Sistema de Visión...")
        os.system("pkill -f 'python -m vision.sistema'")
        subprocess.Popen(["vision-system/.venv/bin/python", "-m", "vision.sistema"])
        return
        
    elif cmd == 'RESTART_BRAIN':
        web_log("SYS", "Reiniciando hilo del Cerebro...")
        if 'brain_thread' in globals() and brain_thread.is_alive():
            web_log("ERR", "El cerebro ya esta corriendo.")
        else:
            brain_instance = CentralBrain()
            brain_thread = threading.Thread(target=brain_instance.run, daemon=True)
            brain_thread.start()
            web_log("SYS", "Cerebro reiniciado exitosamente.")
        return

    if cmd == 'IDLE':
        brain_instance.mode = "IDLE"
        brain_instance.assignments = {10: -1, 11: -1}
        brain_instance.states = {10: "IDLE", 11: "IDLE"}
        for r in [10, 11]:
            brain_instance.udp_sock.sendto(f"{r}:r".encode(), ("192.168.88.255", 8889))
        web_log("BRAIN", "MODO CAMBIADO A: IDLE (Abort Broadcast)")
    else:
        brain_instance.mode = cmd
        web_log("BRAIN", f"MODO CAMBIADO A: {cmd}")

class CentralBrain:

    def __init__(self):
        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            self.udp_sock.bind(("192.168.88.11", 0))
        except:
            pass
        self.last_map_print = 0
        
        # Finite State Machines para los rovers
        # Estados: TURN_PRE, DRIVE_PRE, TURN_APPROACH, SENSOR_APPROACH, TURN_PUSH, DRIVE_PUSH
        self.states = {10: "TURN_PRE", 11: "TURN_PRE"}
        self.assignments = {10: -1, 11: -1}
        self.last_targets = {10: -1, 11: -1}
        self.home_positions = {10: None, 11: None}
        self.has_worked = {10: False, 11: False}
        self.mode = "IDLE"
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind(("0.0.0.0", 8890))
        self.cmd_sock.setblocking(False)

    def connect_vision(self):
        while True:
            try:
                self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_sock.connect((VISION_HOST, VISION_PORT))
                print(f"[BRAIN] Conectado a Vision System en {VISION_HOST}:{VISION_PORT}")
                break
            except Exception as e:
                print(f"[BRAIN] Esperando a Vision System... {e}")
                time.sleep(2)

    def is_on_line_segment(self, px, py, x1, y1, x2, y2):
        if x1 < 0 or y1 < 0 or x2 < 0 or y2 < 0: return False
        L2 = (x2 - x1)**2 + (y2 - y1)**2
        if L2 == 0.0: return False
        t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / L2
        t = max(0.0, min(1.0, t))
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2) <= 0.6

    def valid_cubes(self, msg):
        # Ignora fantasmas fuera de la cancha. compute_assignment y compute_routes
        # DEBEN usar esta misma lista: los índices asignados apuntan a ella.
        grid_cols = msg.get("grid", {}).get("cols", 43.0)
        grid_rows = msg.get("grid", {}).get("rows", 43.0)
        return [c for c in msg.get("cubes", [])
                if 0 <= c.get("col", -1) <= grid_cols and 0 <= c.get("row", -1) <= grid_rows]

    def compute_assignment(self, msg):
        cubes = self.valid_cubes(msg)
        rovers = msg.get("rovers", [])
        
        rx_10, ry_10 = 999.0, 999.0
        rx_11, ry_11 = 999.0, 999.0
        for r in rovers:
            if r["id"] == 10: rx_10, ry_10 = r["col"], r["row"]
            elif r["id"] == 11: rx_11, ry_11 = r["col"], r["row"]
                
        depots = msg.get("depots", [])
        
        # 1. Identificar cubos "activos" (los que todavia no estan en su deposito)
        active_cubes = []
        for i, c in enumerate(cubes):
            color = c.get("color", "")
            depot = next((d for d in depots if d.get("color", "").upper() == color.upper()), None)
            in_depot = False
            if depot and "col" in depot:
                dist_dpt = math.sqrt((c["col"] - depot["col"])**2 + (c["row"] - depot["row"])**2)
                if dist_dpt < 2.0: in_depot = True
            
            if not in_depot:
                active_cubes.append((i, c))

        best_10, best_11 = -1, -1
        mode = getattr(self, "mode", "IDLE")

        # 2. Asignacion MODO AISLADO (e1) - Solo Rover 10
        if mode == "EXEC_10":
            if active_cubes:
                best_dist = 99999.0
                for orig_idx, c in active_cubes:
                    d = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                    if self.assignments[10] == orig_idx: d *= 0.1 # Inercia
                    if d < best_dist:
                        best_dist = d
                        best_10 = orig_idx

        # 3. Asignacion MODO AISLADO (e2) - Solo Rover 11
        elif mode == "EXEC_11":
            if active_cubes:
                best_dist = 99999.0
                for orig_idx, c in active_cubes:
                    d = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2) if rx_11 < 900 else 9999.0
                    if self.assignments[11] == orig_idx: d *= 0.1
                    if d < best_dist:
                        best_dist = d
                        best_11 = orig_idx

        # 4. Asignacion MODO PAREJA (ee o AUTO)
        else:
            if len(active_cubes) == 1:
                orig_idx, c = active_cubes[0]
                d10 = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                d11 = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2) if rx_11 < 900 else 9999.0
                if self.assignments[10] == orig_idx: d10 *= 0.1
                if self.assignments[11] == orig_idx: d11 *= 0.1
                if d10 <= d11: best_10 = orig_idx
                else: best_11 = orig_idx
            
            elif len(active_cubes) >= 2:
                min_total = 99999.0
                for idx10, c10 in active_cubes:
                    for idx11, c11 in active_cubes:
                        if idx10 == idx11: continue
                        d10 = math.sqrt((c10["col"] - rx_10)**2 + (c10["row"] - ry_10)**2) if rx_10 < 900 else 9999.0
                        d11 = math.sqrt((c11["col"] - rx_11)**2 + (c11["row"] - ry_11)**2) if rx_11 < 900 else 9999.0
                        if self.assignments[10] == idx10: d10 *= 0.1
                        if self.assignments[11] == idx11: d11 *= 0.1
                        
                        if d10 + d11 < min_total:
                            min_total = d10 + d11
                            best_10 = idx10
                            best_11 = idx11

        self.assignments[10] = best_10
        self.assignments[11] = best_11
        return best_10, best_11, rx_10, ry_10, rx_11, ry_11

    def compute_routes(self, msg, best_10, best_11, rx_10, ry_10, rx_11, ry_11):
        cubes = self.valid_cubes(msg)
        depots = msg.get("depots", [])

        routes = {10: None, 11: None}
        grid_cols = msg["grid"]["cols"]
        grid_rows = msg["grid"]["rows"]
        
        def calc_route(rid, rx, ry, cube_idx):
            if cube_idx == -1 or rx > 900.0: return None
            if cube_idx >= len(cubes): return None
            c = cubes[cube_idx]
            color = c["color"]
            d = None
            for dpt in depots:
                if dpt.get("color", "").upper() == color.upper():
                    d = dpt
                    break
            if not d: return None
            
            dx = c["col"] - d["col"]
            dy = c["row"] - d["row"]
            length = math.sqrt(dx**2 + dy**2) or 1.0
            
            pre_x = c["col"] + (dx/length) * PRE_APPROACH_DIST
            pre_y = c["row"] + (dy/length) * PRE_APPROACH_DIST
            pre_x = max(3.0, min(grid_cols - 3.0, pre_x))
            pre_y = max(3.0, min(grid_rows - 3.0, pre_y))
            
            # El objetivo de push debe estar MAS ALLA del centro del deposito
            push_x = d["col"] - (dx/length) * 3.0
            push_y = d["row"] - (dy/length) * 3.0
            
            p_avoid = None
            # Check collision with OTHER rover
            other_rx = rx_11 if rid == 10 else rx_10
            other_ry = ry_11 if rid == 10 else ry_10
            
            if other_rx < 900.0:
                # Vector de p1 a p2
                vx = pre_x - rx
                vy = pre_y - ry
                v_len = math.sqrt(vx**2 + vy**2)
                if v_len > 0:
                    vx /= v_len
                    vy /= v_len
                    # Proyeccion del obstaculo en la recta
                    wx = other_rx - rx
                    wy = other_ry - ry
                    proj = wx*vx + wy*vy
                    
                    # Si el obstaculo esta ENTRE el rover y su destino
                    if 0 < proj < v_len:
                        closest_x = rx + proj * vx
                        closest_y = ry + proj * vy
                        dist_to_line = math.sqrt((other_rx - closest_x)**2 + (other_ry - closest_y)**2)
                        
                        # Si esta demasiado cerca de la linea de trayectoria (e.g. 10.0 unidades, considerando las pinzas de 4 bloques)
                        if dist_to_line < 10.0:
                            # Calcular un punto de evasion lateral
                            # Vector perpendicular
                            nx = -vy
                            ny = vx
                            
                            # Decidir a que lado desviar (el que requiera menor desvio)
                            if (wx*nx + wy*ny) > 0:
                                # Obstaculo esta "a la derecha" (direccion normal), evadir hacia la izquierda (-normal)
                                nx = -nx
                                ny = -ny
                                
                            p_avoid_x = closest_x + nx * 10.0
                            p_avoid_y = closest_y + ny * 10.0
                            
                            # Restringir a los bordes de la cancha
                            p_avoid_x = max(3.0, min(grid_cols - 3.0, p_avoid_x))
                            p_avoid_y = max(3.0, min(grid_rows - 3.0, p_avoid_y))
                            
                            p_avoid = (p_avoid_x, p_avoid_y)
                            
            return {
                "p1": (rx, ry),
                "p_avoid": p_avoid,
                "p2": (pre_x, pre_y),
                "p3": (d["col"], d["row"]),
                "cube": (c["col"], c["row"]),
                "push": (push_x, push_y)
            }

        routes[10] = calc_route(10, rx_10, ry_10, best_10)
        routes[11] = calc_route(11, rx_11, ry_11, best_11)
        return routes

    def print_map(self, msg, routes):
        if time.time() - getattr(self, 'last_web_map', 0) > 0.1: # 10Hz
            self.last_web_map = time.time()
            rt = {}
            for rid in [10, 11]:
                if routes.get(rid):
                    r = routes[rid]
                    rt[str(rid)] = {
                        "p1": [r["p1"][0], r["p1"][1]], 
                        "p_avoid": [r["p_avoid"][0], r["p_avoid"][1]] if r.get("p_avoid") else None,
                        "p2": [r["p2"][0], r["p2"][1]], 
                        "p3": [r["p3"][0], r["p3"][1]]
                    }
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
        self.last_map_print = time.time()
        
        scale = SCALE
        grid = msg["grid"]
        map_w = round(grid["cols"] / scale)
        map_h = round(grid["rows"] / scale)
        margin = round(4.0 / scale)
        
        out = "\033[H\033[2J" # Clear screen
        out += "=== CEREBRO DIGITAL (CONTROL CENTRALIZADO) ===\n"
        out += " LEYENDA:\n"
        out += "  Objetivos: [\033[31m R/r \033[0m: Rojo] [\033[32m G/g \033[0m: Verde] [\033[34m B/b \033[0m: Azul] (Mayus=Cubo, Minus=Deposito)\n"
        out += "  Navegacion: [\033[33m > \033[0m: Rover 10] [\033[35m > \033[0m: Rover 11]  [*]: Ruta  [+]: Punto de Aproximacion\n"

        out += "+" + "--"*map_w + "+\n"
        
        depots = {0: 'r', 1: 'g', 2: 'b'}
        cubes_colors = {0: 'R', 1: 'G', 2: 'B'}
        
        for y in range(map_h):
            out += "|"
            current_ansi = "\033[0m"
            for x in range(map_w):
                c = '.'
                in_margin = (x < margin or x >= map_w - margin or y < margin or y >= map_h - margin)
                if in_margin: c = '#'
                
                for d in msg.get("depots", []):
                    if "col" in d and round(d["col"]/scale) == x and round(d["row"]/scale) == y:
                        c = d["color"][0].lower() if d.get("color") else 'd'
                for cb in msg.get("cubes", []):
                    if "col" in cb and round(cb["col"]/scale) == x and round(cb["row"]/scale) == y:
                        c = cb["color"][0].upper() if cb.get("color") else 'C'
                            
                cx, cy = x * scale, y * scale
                ansi = "\033[0m"
                is_path = False
                
                r10 = routes[10]
                if r10 and (self.is_on_line_segment(cx, cy, r10["p1"][0], r10["p1"][1], r10["p2"][0], r10["p2"][1]) or 
                            self.is_on_line_segment(cx, cy, r10["p2"][0], r10["p2"][1], r10["p3"][0], r10["p3"][1])):
                    if c in ['.', '#']:
                        c = '*'
                        if round(r10["p2"][0]/scale) == x and round(r10["p2"][1]/scale) == y: c = '+'
                        ansi = "\033[33m"
                        is_path = True
                        
                r11 = routes[11]
                if r11 and not is_path and (self.is_on_line_segment(cx, cy, r11["p1"][0], r11["p1"][1], r11["p2"][0], r11["p2"][1]) or 
                            self.is_on_line_segment(cx, cy, r11["p2"][0], r11["p2"][1], r11["p3"][0], r11["p3"][1])):
                    if c in ['.', '#']:
                        c = '*'
                        if round(r11["p2"][0]/scale) == x and round(r11["p2"][1]/scale) == y: c = '+'
                        ansi = "\033[35m"
                        is_path = True

                for r in msg.get("rovers", []):
                    if "col" in r and round(r["col"]/scale) == x and round(r["row"]/scale) == y:
                        h = r.get("heading", 0)
                        if h >= 315 or h < 45: c = '>'
                        elif h >= 45 and h < 135: c = '^'
                        elif h >= 135 and h < 225: c = '<'
                        else: c = 'v'
                        ansi = "\033[1;37m"
                        if r["id"] == 10: ansi = "\033[33m"
                        if r["id"] == 11: ansi = "\033[35m"
                
                if c in ['R', 'r']: ansi = "\033[31m"
                elif c in ['G', 'g']: ansi = "\033[32m"
                elif c in ['B', 'b']: ansi = "\033[34m"
                
                if ansi != current_ansi:
                    out += ansi
                    current_ansi = ansi
                    
                out += c
                if in_margin and not is_path and c == '#': out += '#'
                else: out += ' '
                
            if current_ansi != "\033[0m":
                out += "\033[0m"
            out += "|\n"
            
        out += "+" + "--"*map_w + "+\n"
        out += "ROVER 10: {} | ROVER 11: {}\n".format(self.states[10], self.states[11])
        print(out, end="")

    def process_fsm(self, msg, routes):
        # Implementacion del Joystick Invisible basado en el estado
        rovers = {r["id"]: r for r in msg.get("rovers", [])}
        
        for rid in [10, 11]:
            rover = rovers.get(rid)
            route = routes.get(rid)
            
            if not rover: continue
                
            current_target = self.assignments[rid]
            state = self.states[rid]
            
            # Reset state if assignment changed, PERO no interrumpir el Backup
            if state not in ["DRIVE_BACKUP", "TURN_HOME", "DRIVE_HOME", "PARKED"]:
                # Check if color actually changed
                target_changed = True
                curr_color = route.get("color") if route else None
                last_col_attr = f"last_color_{rid}"
                last_color = getattr(self, last_col_attr, None)
                
                if current_target != -1 and self.last_targets[rid] != -1:
                    if curr_color and curr_color == last_color:
                        target_changed = False
                        
                if curr_color:
                    setattr(self, last_col_attr, curr_color)
                    
                if current_target != self.last_targets[rid] and target_changed:
                    if current_target != -1:
                        self.states[rid] = "TURN_PRE"
                    else:
                        self.states[rid] = "TURN_HOME"
                    state = self.states[rid]
                self.last_targets[rid] = current_target
                    
            if current_target == -1 and state not in ["DRIVE_BACKUP", "TURN_HOME", "DRIVE_HOME", "PARKED"]:
                self.states[rid] = "TURN_HOME"
                state = "TURN_HOME"
                
            x, y = rover["col"], rover["row"]
            h = rover["theta"]
            
            # Determinar objetivo segun estado
            if state in ["TURN_HOME", "DRIVE_HOME"]:
                target_x, target_y = msg.get("start", {"col": 5.0, "row": 5.0})["col"], msg.get("start", {"col": 5.0, "row": 5.0})["row"]
            elif route:
                target_x, target_y = route["p2"][0], route["p2"][1] # Default: Pre-approach
                
                # Dinamica de evasion: Si hay punto de desvio y no hemos llegado, ir alla primero
                if route.get("p_avoid") and state in ["TURN_PRE", "DRIVE_PRE"]:
                    dist_avoid = math.sqrt((route["p_avoid"][0]-x)**2 + (route["p_avoid"][1]-y)**2)
                    if dist_avoid > 3.0:
                        target_x, target_y = route["p_avoid"][0], route["p_avoid"][1]
                        
                if state in ["TURN_APPROACH", "SENSOR_APPROACH"]:
                    target_x, target_y = route["cube"][0], route["cube"][1]
                elif state in ["TURN_PUSH", "DRIVE_PUSH", "DRIVE_BACKUP"]:
                    target_x, target_y = route["push"][0], route["push"][1]
            else:
                self.udp_sock.sendto(f"{rid}:M,0,0".encode(), (ROVER_BCAST, ROVER_PORT))
                continue
                
            dx = target_x - x
            dy = target_y - y
            dist = math.sqrt(dx**2 + dy**2)
            
            target_h = math.atan2(-dy, dx) * 180.0 / math.pi
            if target_h < 0: target_h += 360.0
            
            diff = target_h - h
            while diff <= -180.0: diff += 360.0
            while diff > 180.0: diff -= 360.0
            
            ml, mr = 0.0, 0.0
            
            if state in ["TURN_PRE", "TURN_APPROACH", "TURN_PUSH", "TURN_HOME"]:
                if abs(diff) < 20.0:
                    # Transicion a Drive
                    if state == "TURN_PRE": self.states[rid] = "DRIVE_PRE"
                    elif state == "TURN_APPROACH": self.states[rid] = "SENSOR_APPROACH"
                    elif state == "TURN_PUSH": self.states[rid] = "DRIVE_PUSH"
                    elif state == "TURN_HOME": self.states[rid] = "DRIVE_HOME"
                else:
                    turn_speed = diff * 0.008
                    if 0 < turn_speed < 0.30: turn_speed = 0.30
                    if 0 > turn_speed > -0.30: turn_speed = -0.30
                    turn_speed = max(-0.45, min(0.45, turn_speed)) # Pivot max speed aumentado para vencer friccion
                    ml = -turn_speed
                    mr = turn_speed
            
            elif state in ["DRIVE_PRE", "SENSOR_APPROACH", "DRIVE_PUSH", "DRIVE_HOME"]:
                # Check arrival
                dot_prod = dx * math.cos(h * math.pi / 180.0) + dy * -math.sin(h * math.pi / 180.0)
                arrived = (dist < 4.0) or (dist < 6.0 and dot_prod < 0)
                
                if arrived:
                    ml, mr = 0.0, 0.0
                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": 
                        self.states[rid] = "DONE"
                        self.last_targets[rid] = -1 # Force re-assignment next frame
                    elif state == "DRIVE_HOME":
                        self.states[rid] = "PARKED" 
                else:
                    fwd_speed = 0.6 # Velocidad base aumentada para mejor respuesta
                    corr = diff * 0.025
                    corr = max(-0.3, min(0.3, corr))
                    ml = fwd_speed - corr
                    mr = fwd_speed + corr
                    ml = max(-1.0, min(1.0, ml))
                    mr = max(-1.0, min(1.0, mr))
            
            # Enviar comando de joystick invisible al ESP32
            cmd = f"{rid}:M,{ml:.2f},{mr:.2f}".encode()
            self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
            try: web_log('TX', cmd.decode('utf-8'))
            except: pass

    def process_ui(self):
        try:
            data, addr = self.cmd_sock.recvfrom(1024)
            cmd = data.decode('utf-8').strip()
            if cmd.startswith("B:"):
                self.mode = cmd[2:]
                # Reiniciar estados al cambiar modo
                self.states = {10: "TURN_PRE", 11: "TURN_PRE"}
                print(f"\n[BRAIN] MODO CAMBIADO A: {self.mode}\n")
        except BlockingIOError:
            pass
        except Exception:
            pass



    def process_waypoints(self, msg, routes):
        print(f"[DEBUG] process_waypoints called! Mode: {self.mode}")
        # Pura navegación por coordenadas
        rovers = {r["id"]: r for r in msg.get("rovers", [])}
        import time
        if not hasattr(self, "last_cmd_time"): self.last_cmd_time = {10: 0, 11: 0}
        now = time.time()
        
        for rid in [10, 11]:
            rover = rovers.get(rid)
            route = routes.get(rid)
            if not rover: continue
            
            target_idx = self.assignments[rid]
            print(f"[DEBUG] Rover {rid} target_idx={target_idx}, route={bool(route)}")
            if target_idx == -1:
                # Inactivo / IDLE
                if now - self.last_cmd_time.get(rid, 0) > 0.5:
                    self.udp_sock.sendto(f"{rid}:r".encode(), (ROVER_BCAST, ROVER_PORT))
                    self.udp_sock.sendto(f"{rid}:M,0,0".encode(), (ROVER_BCAST, ROVER_PORT))
                    self.last_cmd_time[rid] = now
                if self.states.get(rid) != "PARKED":
                    self.states[rid] = "PARKED"
                continue
                
            if not route: continue
            
            # Máquina de estados de Waypoints
            state = self.states.get(rid, "WP_AVOID")
            x, y = rover["col"], rover["row"]
            
            # Lista secuencial de waypoints
            waypoints = []
            if route.get("p_avoid"):
                dist_avoid = math.sqrt((x - route["p_avoid"][0])**2 + (y - route["p_avoid"][1])**2)
                if dist_avoid > 3.0 and state == "WP_AVOID":
                    waypoints.append(("WP_AVOID", route["p_avoid"]))
            
            waypoints.append(("WP_PRE", route["p2"]))
            
            # Para el push final, vamos más profundo en el depósito
            waypoints.append(("WP_PUSH", route["push"]))
            
            # Inicializar estado si es nuevo
            if state not in [w[0] for w in waypoints]:
                state = waypoints[0][0]
                self.states[rid] = state
            
            for i, (wp_name, wp_pos) in enumerate(waypoints):
                if state == wp_name:
                    # Chequear si ya llegó
                    dist = math.sqrt((x - wp_pos[0])**2 + (y - wp_pos[1])**2)
                    threshold = 3.5 if wp_name != "WP_PUSH" else 2.0
                    
                    if dist < threshold:
                        # Avanzar al siguiente
                        if i + 1 < len(waypoints):
                            next_state = waypoints[i+1][0]
                            self.states[rid] = next_state
                            web_log("BRAIN", f"Rover {rid} alcanzó {wp_name}. Avanzando a {next_state}")
                            self.last_cmd_time[rid] = 0
                        else:
                            # Terminó la secuencia
                            self.states[rid] = "PARKED"
                            self.assignments[rid] = -1
                            web_log("BRAIN", f"Rover {rid} completó ruta.")
                    else:
                        # Enviar coordenada objetivo
                        if now - self.last_cmd_time.get(rid, 0) > 0.4:
                            cmd = f"{rid}:G,{wp_pos[0]:.2f},{wp_pos[1]:.2f}".encode()
                            self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                            try: web_log('TX', f"WP {wp_name}: {cmd.decode('utf-8')}")
                            except: pass
                            self.last_cmd_time[rid] = now
                    break


    def process_delegated(self, msg, routes):
        # Solucion Delegada Total: El Cerebro solo evalua el estado y delega al C++ del ESP32.
        raw_cubes = msg.get("cubes", [])
        grid_cols = msg.get("grid", {}).get("cols", 43.0)
        grid_rows = msg.get("grid", {}).get("rows", 28.0)
        cubes = [c for c in raw_cubes if 0 <= c.get("col", -1) <= grid_cols and 0 <= c.get("row", -1) <= grid_rows]
        
        import time
        if not hasattr(self, "last_cmd_time"): self.last_cmd_time = {10: 0, 11: 0}
        now = time.time()
        
        for rid in [10, 11]:
            target_idx = self.assignments[rid]
            print(f"[DEBUG] Rover {rid} target_idx={target_idx}, route={bool(route)}")
            if target_idx >= len(cubes): target_idx = -1
            
            if target_idx == -1:
                # Si el rover no tiene tarea (ej. MODO EXEC_10 para el R11), detenerlo.
                # NO ENVIARLO A CASA PARA NO CONFUNDIR AL USUARIO
                if self.states.get(rid) != "PARKED":
                    self.udp_sock.sendto(f"{rid}:M,0,0".encode(), (ROVER_BCAST, ROVER_PORT))
                    self.states[rid] = "PARKED"
                continue
                
            # Va a cazar.
            color_str = cubes[target_idx].get("color", "").upper()
            cmd_char = None
            if color_str == "RED": cmd_char = '3'
            elif color_str == "GREEN": cmd_char = '4'
            elif color_str == "BLUE": cmd_char = '5'
            
            if cmd_char:
                # Enviar comando de alto nivel dirigido al rover (ej. "10:3")
                # El rover usara su IMU Fusion + APF C++ internamente.
                if now - self.last_cmd_time.get(rid, 0) > 0.5 or self.states.get(rid) != f"HUNT_{color_str}":
                    cmd = f"{rid}:{cmd_char}".encode()
                    self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                    try: web_log('TX', f"Delegando a ESP32: {cmd.decode('utf-8')}")
                    except: pass
                    self.last_cmd_time[rid] = now
                    self.states[rid] = f"HUNT_{color_str}"

    def process_hybrid(self, msg):

        # Solucion Hibrida: El Cerebro evalua el estado global y asigna tareas de alto nivel.
        # Los ESP32 ejecutan la maniobra local (hunt_cube) usando sus sensores fisicos para precision cero-latencia.
        raw_cubes = msg.get("cubes", [])
        grid_cols = msg.get("grid", {}).get("cols", 43.0)
        grid_rows = msg.get("grid", {}).get("rows", 28.0)
        # Ignorar fantasmas fuera de la cancha
        cubes = [c for c in raw_cubes if 0 <= c.get("col", -1) <= grid_cols and 0 <= c.get("row", -1) <= grid_rows]
        
        for rid in [10, 11]:
            target_idx = self.assignments[rid]
            print(f"[DEBUG] Rover {rid} target_idx={target_idx}, route={bool(route)}")
            if target_idx >= len(cubes): target_idx = -1
            
            if target_idx == -1:
                # No hay cubos asignados.
                prev_state = self.states.get(rid, "")
                if prev_state.startswith("HUNT_"):
                    lost_color = prev_state.split("_")[1]
                    try: web_log("ERR", f"R{rid} PERDIO DE VISTA EL COLOR {lost_color}")
                    except: pass
                    self.states[rid] = f"LOST_{lost_color}"
                    self.udp_sock.sendto(f"{rid}:r".encode(), (ROVER_BCAST, ROVER_PORT))
                    continue
                
                if self.has_worked[rid] and not prev_state.startswith("LOST_") and prev_state != "PARKED":
                    hx, hy = self.home_positions[rid] if self.home_positions[rid] else (5.0, 21.5)
                    self.udp_sock.sendto(f"{rid}:H,{hx:.2f},{hy:.2f}".encode(), (ROVER_BCAST, ROVER_PORT))
                    self.states[rid] = "PARKED"
                continue
                
            if target_idx < len(cubes):
                # Va a cazar.
                rover_data = next((r for r in msg.get("rovers", []) if r.get("id") == rid), None)
                if not rover_data: continue
                
                if self.home_positions[rid] is None:
                    self.home_positions[rid] = (rover_data.get("col", 5.0), rover_data.get("row", 21.5))
                self.has_worked[rid] = True
                color_str = cubes[target_idx].get("color", "").upper()
                
                cmd_char = None
                if color_str == "RED": cmd_char = '3'
                elif color_str == "GREEN": cmd_char = '4'
                elif color_str == "BLUE": cmd_char = '5'
                
                # OBTENER RUTA PYTHON (con avoidance)
                routes = getattr(self, 'last_computed_routes', None)
                r = routes.get(rid) if routes else None
                
                # True Hybrid: Conducir por Python hasta el pre-approach, luego delegar al ESP32
                if r and cmd_char:
                    x, y = rover_data["col"], rover_data["row"]
                    h = rover_data["theta"]
                    
                    # FASE LARGA: Python dirige evadiendo hacia el Pre-Approach (p2)
                    target_x, target_y = r["p2"][0], r["p2"][1]
                    dist_to_p2 = ((target_x-x)**2 + (target_y-y)**2)**0.5
                    
                    # Si llego al pre-approach (p2), soltar al ESP32 para el empuje final!
                    if dist_to_p2 < 3.0 and not (r.get("p_avoid") and ((r["p_avoid"][0]-x)**2 + (r["p_avoid"][1]-y)**2)**0.5 > 3.0):
                        import time
                        if not hasattr(self, "last_cmd_time"): self.last_cmd_time = {10: 0, 11: 0}
                        now = time.time()
                        if now - self.last_cmd_time.get(rid, 0) > 0.5 or self.states.get(rid) != f"HUNT_{color_str}":
                            cmd = f"{rid}:{cmd_char}".encode()
                            self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                            try: web_log('TX', f"Fase Final ESP32: {cmd.decode('utf-8')}")
                            except: pass
                            self.last_cmd_time[rid] = now
                        self.states[rid] = f"HUNT_{color_str}"
                        continue
                        
                    if r.get("p_avoid"):
                        dist_avoid = ((r["p_avoid"][0]-x)**2 + (r["p_avoid"][1]-y)**2)**0.5
                        if dist_avoid > 4.0:
                            target_x, target_y = r["p_avoid"][0], r["p_avoid"][1]
                            self.states[rid] = "AVOID_MODE"
                        else:
                            self.states[rid] = "DRIVE_PRE"
                    else:
                        self.states[rid] = "DRIVE_PRE"
                        
                    import math
                    dx = target_x - x
                    dy = target_y - y
                    target_h = math.atan2(-dy, dx) * 180.0 / math.pi
                    if target_h < 0: target_h += 360.0
                    
                    diff = target_h - h
                    while diff <= -180.0: diff += 360.0
                    while diff > 180.0: diff -= 360.0
                    
                    if abs(diff) > 25.0:
                        # Pivotear
                        turn_speed = diff * 0.005
                        if 0 < turn_speed < 0.16: turn_speed = 0.16
                        if 0 > turn_speed > -0.16: turn_speed = -0.16
                        turn_speed = max(-0.25, min(0.25, turn_speed))
                        ml, mr = -turn_speed, turn_speed
                    else:
                        # Avanzar corrigiendo
                        fwd = 0.5
                        corr = diff * 0.015
                        corr = max(-0.2, min(0.2, corr))
                        ml = fwd - corr
                        mr = fwd + corr
                        
                    ml = max(-1.0, min(1.0, ml))
                    mr = max(-1.0, min(1.0, mr))
                    cmd = f"{rid}:M,{ml:.2f},{mr:.2f}".encode()
                    self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                    
                    import time
                    now = time.time()
                    if not hasattr(self, "last_tx_log"): self.last_tx_log = 0
                    if now - self.last_tx_log > 2.0:
                        try: web_log('TX', f"[{rid}] Python Control: M,{ml:.2f},{mr:.2f}")
                        except: pass
                        self.last_tx_log = now

    def run(self):
        self.connect_vision()
        buffer = ""
        while True:
            try:
                data = self.tcp_sock.recv(4096).decode('utf-8')
                if not data: 
                    print("[BRAIN] Desconectado.")
                    self.connect_vision()
                    continue
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        msg = json.loads(line)
                        self.process_ui()
                        
                        if self.mode == "IDLE":
                            # Enviar comandos de parada constantemente
                            import time
                            now = time.time()
                            if not hasattr(self, "last_idle_time"): self.last_idle_time = 0
                            if now - self.last_idle_time > 0.5:
                                self.udp_sock.sendto(b"10:r", ("192.168.88.255", 8889))
                                self.udp_sock.sendto(b"10:M,0,0", ("192.168.88.255", 8889))
                                self.udp_sock.sendto(b"11:r", ("192.168.88.255", 8889))
                                self.udp_sock.sendto(b"11:M,0,0", ("192.168.88.255", 8889))
                                self.last_idle_time = now
                            self.print_map(msg, {10: None, 11: None})
                            continue

                            
                        b10, b11, rx10, ry10, rx11, ry11 = self.compute_assignment(msg)
                        
                        # Filtrar segun modo aislando las asignaciones
                        if self.mode == "EXEC_10": 
                            b11 = -1
                            self.assignments[11] = -1
                        if self.mode == "EXEC_11": 
                            b10 = -1
                            self.assignments[10] = -1
                        
                        routes = self.compute_routes(msg, b10, b11, rx10, ry10, rx11, ry11)
                        self.last_computed_routes = routes
                        self.print_map(msg, routes)
                        
                        print(f"[DEBUG] check EXEC in run. mode={self.mode}")
                        if "EXEC" in self.mode or self.mode == "AUTO":
                            self.process_waypoints(msg, routes)
            except KeyboardInterrupt:
                break
            except Exception as e:
                import traceback
                traceback.print_exc()
                time.sleep(1)

if __name__ == "__main__":
    global brain_instance
    brain_instance = CentralBrain()
    
    # Run the brain in a daemon thread
    global brain_thread
    brain_thread = threading.Thread(target=brain_instance.run, daemon=True)
    brain_thread.start()
    
    print("[WEB] Servidor Dashboard iniciado en http://0.0.0.0:8891")
    socketio.run(app, host='0.0.0.0', port=8891, debug=False, use_reloader=False)
