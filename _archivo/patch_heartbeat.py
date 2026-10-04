import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Rate-limit the heartbeat to avoid flooding the ESP32
    # Instead of sending every frame (30Hz), send only once per second per rover
    
    # We will inject a heartbeat rate limiter into central_brain.py
    # Let's find process_hybrid
    
    old_cmd = """                    cmd = f"{rid}:{cmd_char}".encode()
                    self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                    self.states[rid] = f"HUNT_{color_str}" """
                    
    new_cmd = """                    cmd = f"{rid}:{cmd_char}".encode()
                    
                    # Rate limit: Enviar maximo 2 veces por segundo para no saturar al ESP32
                    import time
                    if not hasattr(self, "last_cmd_time"): self.last_cmd_time = {10: 0, 11: 0}
                    
                    now = time.time()
                    if now - self.last_cmd_time.get(rid, 0) > 0.5 or self.states.get(rid) != f"HUNT_{color_str}":
                        self.udp_sock.sendto(cmd, (ROVER_BCAST, ROVER_PORT))
                        self.last_cmd_time[rid] = now
                        
                    self.states[rid] = f"HUNT_{color_str}" """
                    
    content = content.replace(old_cmd, new_cmd)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Rate limit patched")
