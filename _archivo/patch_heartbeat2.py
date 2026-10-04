import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Rate-limit the heartbeat to avoid flooding the ESP32
    old_cmd = """                    cmd = f"{rid}:{cmd_char}".encode()
                    self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                    
                    # Actualizar estado visual
                    self.states[rid] = f"HUNT_{color_str}\""""
                    
    new_cmd = """                    cmd = f"{rid}:{cmd_char}".encode()
                    
                    # Rate limit: Enviar maximo 2 veces por segundo para no saturar buffer UDP
                    import time
                    if not hasattr(self, "last_cmd_time"): self.last_cmd_time = {10: 0, 11: 0}
                    now = time.time()
                    if now - self.last_cmd_time.get(rid, 0) > 0.5 or self.states.get(rid) != f"HUNT_{color_str}":
                        self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                        self.last_cmd_time[rid] = now
                    
                    # Actualizar estado visual
                    self.states[rid] = f"HUNT_{color_str}\""""
                    
    content = content.replace(old_cmd, new_cmd)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Heartbeat rate limit successfully applied")
