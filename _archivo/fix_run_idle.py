with open("central_brain.py", "r") as f:
    content = f.read()

# Restore the IDLE check inside run()
idle_code = """
                        if self.mode == "IDLE":
                            # Enviar comandos de parada constantemente
                            import time
                            now = time.time()
                            if not hasattr(self, "last_idle_time"): self.last_idle_time = 0
                            if now - self.last_idle_time > 0.5:
                                self.udp_sock.sendto(b"10:r", ("255.255.255.255", 8889))
                                self.udp_sock.sendto(b"10:M,0,0", ("255.255.255.255", 8889))
                                self.udp_sock.sendto(b"11:r", ("255.255.255.255", 8889))
                                self.udp_sock.sendto(b"11:M,0,0", ("255.255.255.255", 8889))
                                self.last_idle_time = now
                            self.print_map(msg, {10: None, 11: None})
                            continue
"""

content = content.replace("# IDLE block removed to allow process_waypoints to stop rovers", idle_code)

# Restore the EXEC check
content = content.replace("self.process_waypoints(msg, routes)", 
"""                        if "EXEC" in self.mode or self.mode == "AUTO":
                            self.process_waypoints(msg, routes)""")

with open("central_brain.py", "w") as f:
    f.write(content)
