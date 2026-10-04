import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_block = """            if target_idx == -1:
                # No hay cubos asignados. Solo va a casa si TRABAJO.
                if self.has_worked[rid] and self.states[rid] != "PARKED":
                    hx, hy = self.home_positions[rid] if self.home_positions[rid] else (5.0, 21.5)
                    self.udp_sock.sendto(f"{rid}:H,{hx:.2f},{hy:.2f}".encode(), (ROVER_BCAST, ROVER_PORT))
                    self.states[rid] = "PARKED"
                continue"""
                
    new_block = """            if target_idx == -1:
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
                continue"""
                
    content = content.replace(old_block, new_block)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Added LOST_COLOR state to Brain")
