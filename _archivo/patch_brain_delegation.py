import re

with open("central_brain.py", "r") as f:
    code = f.read()

# Make it use process_delegated
code = code.replace("self.process_fsm(msg, routes)", "self.process_delegated(msg, routes)")
code = code.replace("self.process_hybrid(msg)", "self.process_delegated(msg, getattr(self, 'last_computed_routes', {}))")

new_func = """
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
"""

if "def process_delegated" not in code:
    code = code.replace("    def process_hybrid(self, msg):", new_func)
    
with open("central_brain.py", "w") as f:
    f.write(code)

print("Patched central_brain.py to delegate completely to C++ logic!")
