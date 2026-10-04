import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Update compute_assignment to ignore cubes that are in the depot
    assign_old = """        for i, c in enumerate(cubes):
            cost_matrix[0][i] = math.sqrt((c["pos"]["x"] - rx_10)**2 + (c["pos"]["y"] - ry_10)**2) if rx_10 < 900 else 9999.0
            cost_matrix[1][i] = math.sqrt((c["pos"]["x"] - rx_11)**2 + (c["pos"]["y"] - ry_11)**2) if rx_11 < 900 else 9999.0"""
            
    assign_new = """        for i, c in enumerate(cubes):
            cost0, cost1 = 9999.0, 9999.0
            if "col" in c and "row" in c:
                # Check if cube is already in a depot (approx dist < 10)
                color = c.get("color", "")
                depot = None
                for dpt in depots:
                    if dpt.get("color", "").upper() == color.upper():
                        depot = dpt
                        break
                in_depot = False
                if depot and "col" in depot:
                    dist_dpt = math.sqrt((c["col"] - depot["col"])**2 + (c["row"] - depot["row"])**2)
                    if dist_dpt < 12.0: in_depot = True
                    
                if not in_depot:
                    if rx_10 < 900: cost0 = math.sqrt((c["col"] - rx_10)**2 + (c["row"] - ry_10)**2)
                    if rx_11 < 900: cost1 = math.sqrt((c["col"] - rx_11)**2 + (c["row"] - ry_11)**2)
            cost_matrix[0][i] = cost0
            cost_matrix[1][i] = cost1"""
            
    content = content.replace(assign_old, assign_new)

    # 2. Fix the FSM to transition to the next cube and reset state when target changes
    # In __init__:
    init_old = """        self.states = {10: "TURN_PRE", 11: "TURN_PRE"}
        self.assignments = {10: -1, 11: -1}
        self.mode = "IDLE" """
        
    init_new = """        self.states = {10: "TURN_PRE", 11: "TURN_PRE"}
        self.assignments = {10: -1, 11: -1}
        self.last_targets = {10: -1, 11: -1}
        self.mode = "IDLE" """
        
    content = content.replace(init_old, init_new)

    # In process_fsm:
    fsm_old = """            if not rover or not route: continue
            
            state = self.states[rid]
            x, y = rover["col"], rover["row"]"""
            
    fsm_new = """            if not rover or not route: 
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
                
            x, y = rover["col"], rover["row"]"""
            
    content = content.replace(fsm_old, fsm_new)

    # Handle DRIVE_PUSH arrival
    push_old = """                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": pass # Hecho!"""
                    
    push_new = """                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": 
                        self.states[rid] = "DONE"
                        self.last_targets[rid] = -1 # Force re-assignment next frame"""
                        
    content = content.replace(push_old, push_new)

    # In compute_assignment, avoid resetting assignment randomly
    # Wait, compute_assignment runs every frame.
    # If a rover is currently assigned a cube, we should add a massive discount so it sticks to it!
    assign_stick_old = """        for c10 in range(3):
            for c11 in range(3):
                if c10 == c11: continue
                total = cost_matrix[0][c10] + cost_matrix[1][c11]
                if total < min_total:
                    min_total = total
                    best_10, best_11 = c10, c11"""
                    
    assign_stick_new = """        for c10 in range(3):
            for c11 in range(3):
                if c10 == c11: continue
                # Añadir inercia/descuento al cubo que ya teníamos asignado
                cost0 = cost_matrix[0][c10]
                cost1 = cost_matrix[1][c11]
                if self.assignments[10] == c10: cost0 *= 0.1 # 90% discount to stick
                if self.assignments[11] == c11: cost1 *= 0.1
                
                total = cost0 + cost1
                if total < min_total:
                    min_total = total
                    best_10, best_11 = c10, c11
                    
        # Check single remaining cubes if peer is DONE
        if best_10 == -1 and cost_matrix[0][0] < 1000:
            for c10 in range(3):
                if cost_matrix[0][c10] < 1000: best_10 = c10
        if best_11 == -1 and cost_matrix[1][0] < 1000:
            for c11 in range(3):
                if cost_matrix[1][c11] < 1000: best_11 = c11"""
                
    content = content.replace(assign_stick_old, assign_stick_new)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
