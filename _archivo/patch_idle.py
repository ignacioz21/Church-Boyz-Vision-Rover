import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_ui = """                    if new_mode in ["EXEC_10", "EXEC_11", "EXEC_ALL", "AUTO"] and self.mode == "IDLE":
                        self.home_positions = {10: None, 11: None}
                        self.has_worked = {10: False, 11: False}
                    self.mode = new_mode"""
                    
    new_ui = """                    if new_mode in ["EXEC_10", "EXEC_11", "EXEC_ALL", "AUTO"] and self.mode == "IDLE":
                        self.home_positions = {10: None, 11: None}
                        self.has_worked = {10: False, 11: False}
                    if new_mode == "IDLE" and self.mode != "IDLE":
                        self.udp_sock.sendto(b"10:r", (ROVER_BCAST, ROVER_PORT))
                        self.udp_sock.sendto(b"11:r", (ROVER_BCAST, ROVER_PORT))
                        self.states[10] = "PARKED"
                        self.states[11] = "PARKED"
                    self.mode = new_mode"""
                    
    content = content.replace(old_ui, new_ui)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("IDLE cancel patched")
