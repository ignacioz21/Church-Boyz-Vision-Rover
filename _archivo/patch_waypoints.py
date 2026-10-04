import re

with open("central_brain.py", "r") as f:
    content = f.read()

# Add process_waypoints
wp_func = """
    def process_waypoints(self, msg, routes):
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

"""

# replace process_delegated call with process_waypoints
content = content.replace("self.process_delegated(msg, routes)", "self.process_waypoints(msg, routes)")

# Insert the function before process_delegated
content = content.replace("    def process_delegated(self, msg, routes):", wp_func + "\n    def process_delegated(self, msg, routes):")

with open("central_brain.py", "w") as f:
    f.write(content)

