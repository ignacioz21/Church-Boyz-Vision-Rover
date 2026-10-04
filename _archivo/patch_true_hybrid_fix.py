import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_hybrid = """                    # Checar si ya estamos en fase final (cerca del objetivo o de pre-approach)
                    dx_cube = r["cube"][0] - x
                    dy_cube = r["cube"][1] - y
                    dist_to_cube = (dx_cube**2 + dy_cube**2)**0.5
                    
                    # Si esta a menos de 10 unidades del cubo, SOLTAR AL ESP32 (Fase Final APF local)
                    if dist_to_cube < 10.0:
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
                        
                    # FASE LARGA: Python dirige evadiendo
                    target_x, target_y = r["p2"][0], r["p2"][1]
                    if r.get("p_avoid"):
                        dist_avoid = ((r["p_avoid"][0]-x)**2 + (r["p_avoid"][1]-y)**2)**0.5
                        if dist_avoid > 3.0:
                            target_x, target_y = r["p_avoid"][0], r["p_avoid"][1]
                            self.states[rid] = "AVOID_MODE"
                        else:
                            self.states[rid] = "DRIVE_PRE"
                    else:
                        self.states[rid] = "DRIVE_PRE\""""
                        
    new_hybrid = """                    # FASE LARGA: Python dirige evadiendo hacia el Pre-Approach (p2)
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
                        self.states[rid] = "DRIVE_PRE\""""
                        
    content = content.replace(old_hybrid, new_hybrid)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Fixed True Hybrid switch condition")
