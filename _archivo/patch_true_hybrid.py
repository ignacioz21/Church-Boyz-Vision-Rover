import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_hybrid = """            if target_idx < len(cubes):
                # Va a cazar. Guardamos su posicion inicial si no la tenemos.
                if self.home_positions[rid] is None:
                    rover_data = next((r for r in msg.get("rovers", []) if r.get("id") == rid), None)
                    if rover_data:
                        self.home_positions[rid] = (rover_data.get("col", 5.0), rover_data.get("row", 21.5))
                self.has_worked[rid] = True
                color_str = cubes[target_idx].get("color", "").upper()
                cmd_char = None
                if color_str == "RED": cmd_char = '3'
                elif color_str == "GREEN": cmd_char = '4'
                elif color_str == "BLUE": cmd_char = '5'
                
                if cmd_char:
                    # Enviar comando de alto nivel dirigido al rover (ej. "10:3")
                    # El rover ignorara el comando si ya esta dentro de la funcion bloqueante hunt_cube()
                    # por lo que enviar esto repetidamente es seguro y actua como un heartbeat de mision.
                    cmd = f"{rid}:{cmd_char}".encode()
                    
                    # Rate limit: Enviar maximo 2 veces por segundo para no saturar buffer UDP
                    import time
                    if not hasattr(self, "last_cmd_time"): self.last_cmd_time = {10: 0, 11: 0}
                    now = time.time()
                    if now - self.last_cmd_time.get(rid, 0) > 0.5 or self.states.get(rid) != f"HUNT_{color_str}":
                        self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                        try: web_log('TX', cmd.decode('utf-8'))
                        except: pass
                        self.last_cmd_time[rid] = now
                    
                    # Actualizar estado visual
                    self.states[rid] = f"HUNT_{color_str}\""""
                    
    new_hybrid = """            if target_idx < len(cubes):
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
                    
                    # Checar si ya estamos en fase final (cerca del objetivo o de pre-approach)
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
                        self.last_tx_log = now"""
                        
    content = content.replace(old_hybrid, new_hybrid)
    
    # Store last_computed_routes in the run loop
    old_run = """                        routes = self.compute_routes(msg, b10, b11, rx10, ry10, rx11, ry11)
                        self.print_map(msg, routes)
                        
                        if "EXEC" in self.mode or self.mode == "AUTO":
                            self.process_hybrid(msg)"""
                            
    new_run = """                        routes = self.compute_routes(msg, b10, b11, rx10, ry10, rx11, ry11)
                        self.last_computed_routes = routes
                        self.print_map(msg, routes)
                        
                        if "EXEC" in self.mode or self.mode == "AUTO":
                            self.process_hybrid(msg)"""
                            
    content = content.replace(old_run, new_run)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("True Hybrid Avoidance injected")
