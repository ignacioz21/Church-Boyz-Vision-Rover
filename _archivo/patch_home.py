import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Update process_fsm top logic to handle route=None and Home states
    fsm_top_old = """            if not rover or not route: 
                self.udp_sock.sendto(f"{rid}:M,0,0".encode(), (ROVER_BCAST, ROVER_PORT))
                continue
                
            # Reset state if assignment changed
            current_target = self.assignments[rid]
            if current_target != self.last_targets[rid]:
                self.states[rid] = "TURN_PRE"
                self.last_targets[rid] = current_target
            
            state = self.states[rid]
            if state == "DONE":
                self.udp_sock.sendto(f"{rid}:M,0,0".encode(), (ROVER_BCAST, ROVER_PORT))
                continue
                
            x, y = rover["col"], rover["row"]
            h = rover["theta"]
            
            # Determinar objetivo segun estado
            target_x, target_y = route["p2"][0], route["p2"][1] # Default: Pre-approach
            if state in ["TURN_APPROACH", "SENSOR_APPROACH"]:
                target_x, target_y = route["cube"][0], route["cube"][1]
            elif state in ["TURN_PUSH", "DRIVE_PUSH"]:
                target_x, target_y = route["push"][0], route["push"][1]"""
                
    fsm_top_new = """            if not rover: continue
                
            current_target = self.assignments[rid]
            state = self.states[rid]
            
            # Reset state if assignment changed, PERO no interrumpir el Backup
            if state not in ["DRIVE_BACKUP", "TURN_HOME", "DRIVE_HOME", "PARKED"]:
                if current_target != self.last_targets[rid]:
                    if current_target != -1:
                        self.states[rid] = "TURN_PRE"
                    else:
                        self.states[rid] = "TURN_HOME"
                    self.last_targets[rid] = current_target
                    state = self.states[rid]
                    
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
                if state in ["TURN_APPROACH", "SENSOR_APPROACH"]:
                    target_x, target_y = route["cube"][0], route["cube"][1]
                elif state in ["TURN_PUSH", "DRIVE_PUSH", "DRIVE_BACKUP"]:
                    target_x, target_y = route["push"][0], route["push"][1]
            else:
                self.udp_sock.sendto(f"{rid}:M,0,0".encode(), (ROVER_BCAST, ROVER_PORT))
                continue"""
                
    content = content.replace(fsm_top_old, fsm_top_new)

    # 2. Update Turn logic to include TURN_HOME
    turn_old = """            if state in ["TURN_PRE", "TURN_APPROACH", "TURN_PUSH"]:
                if abs(diff) < 20.0:
                    # Transicion a Drive
                    if state == "TURN_PRE": self.states[rid] = "DRIVE_PRE"
                    elif state == "TURN_APPROACH": self.states[rid] = "SENSOR_APPROACH"
                    elif state == "TURN_PUSH": self.states[rid] = "DRIVE_PUSH"
                else:"""
                
    turn_new = """            if state in ["TURN_PRE", "TURN_APPROACH", "TURN_PUSH", "TURN_HOME"]:
                if abs(diff) < 20.0:
                    # Transicion a Drive
                    if state == "TURN_PRE": self.states[rid] = "DRIVE_PRE"
                    elif state == "TURN_APPROACH": self.states[rid] = "SENSOR_APPROACH"
                    elif state == "TURN_PUSH": self.states[rid] = "DRIVE_PUSH"
                    elif state == "TURN_HOME": self.states[rid] = "DRIVE_HOME"
                else:"""
                
    content = content.replace(turn_old, turn_new)

    # 3. Update Drive logic to include BACKUP and HOME
    drive_old = """            elif state in ["DRIVE_PRE", "SENSOR_APPROACH", "DRIVE_PUSH"]:
                # Check arrival
                dot_prod = dx * math.cos(h * math.pi / 180.0) + dy * -math.sin(h * math.pi / 180.0)
                arrived = (dist < 4.0) or (dist < 6.0 and dot_prod < 0)
                
                if arrived:
                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": 
                        self.states[rid] = "DONE"
                        self.last_targets[rid] = -1 # Force re-assignment next frame
                else:
                    # Velocidad Proporcional ultra-lenta
                    fwd_speed = 0.25 
                    if dist < 12.0:
                        fwd_speed = max(0.16, 0.25 * (dist / 12.0))
                        
                    if state == "DRIVE_PUSH" and dist < 6.0:
                        fwd_speed = max(0.20, fwd_speed)
                        
                    corr = diff * 0.008
                    corr = max(-0.15, min(0.15, corr))
                    ml = fwd_speed - corr
                    mr = fwd_speed + corr"""
                    
    drive_new = """            elif state in ["DRIVE_PRE", "SENSOR_APPROACH", "DRIVE_PUSH", "DRIVE_HOME"]:
                dot_prod = dx * math.cos(h * math.pi / 180.0) + dy * -math.sin(h * math.pi / 180.0)
                arrived = (dist < 4.0) or (dist < 6.0 and dot_prod < 0)
                
                if arrived:
                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": self.states[rid] = "DRIVE_BACKUP"
                    elif state == "DRIVE_HOME": self.states[rid] = "PARKED"
                else:
                    fwd_speed = 0.25 
                    if dist < 12.0: fwd_speed = max(0.16, 0.25 * (dist / 12.0))
                    if state == "DRIVE_PUSH" and dist < 6.0: fwd_speed = max(0.20, fwd_speed)
                    corr = diff * 0.008
                    corr = max(-0.15, min(0.15, corr))
                    ml = fwd_speed - corr
                    mr = fwd_speed + corr
                    
            elif state == "DRIVE_BACKUP":
                # En backup, el target_x, target_y es el push, así que retrocedemos hasta alejarnos 8.0 celdas
                if dist > 8.0:
                    self.last_targets[rid] = -1 # Forzar re-evaluacion de nueva tarea o ir a casa
                    self.states[rid] = "IDLE_TEMP"
                    ml, mr = 0.0, 0.0
                else:
                    ml, mr = -0.22, -0.22 # Reversa recta
                    
            elif state == "PARKED":
                ml, mr = 0.0, 0.0"""
                
    content = content.replace(drive_old, drive_new)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
