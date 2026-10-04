import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old = """            if target_idx == -1:
                # No hay cubos asignados (termino o no hay)
                self.states[rid] = "STANDBY"
                continue"""
                
    new = """            if target_idx == -1:
                # No hay cubos asignados (termino o no hay)
                if self.states[rid] != "PARKED":
                    self.udp_sock.sendto(f"{rid}:H".encode(), (ROVER_BCAST, ROVER_PORT))
                    self.states[rid] = "PARKED"
                continue"""
                
    content = content.replace(old, new)
    
    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
