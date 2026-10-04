import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Add UDP listener in __init__
    init_old = """        self.states = {10: "TURN_PRE", 11: "TURN_PRE"}
        self.assignments = {10: -1, 11: -1}"""
    
    init_new = """        self.states = {10: "TURN_PRE", 11: "TURN_PRE"}
        self.assignments = {10: -1, 11: -1}
        self.mode = "IDLE"
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind(("0.0.0.0", 8890))
        self.cmd_sock.setblocking(False)"""
        
    content = content.replace(init_old, init_new)

    # 2. Add UI command processing
    ui_cmd = """    def process_ui(self):
        try:
            data, addr = self.cmd_sock.recvfrom(1024)
            cmd = data.decode('utf-8').strip()
            if cmd.startswith("B:"):
                self.mode = cmd[2:]
                # Reiniciar estados al cambiar modo
                self.states = {10: "TURN_PRE", 11: "TURN_PRE"}
                print(f"\\n[BRAIN] MODO CAMBIADO A: {self.mode}\\n")
        except BlockingIOError:
            pass
        except Exception:
            pass

    def run(self):"""
    
    content = content.replace("    def run(self):", ui_cmd)
    
    # 3. Add mode logic in run() loop
    run_loop_old = """                        b10, b11, rx10, ry10, rx11, ry11 = self.compute_assignment(msg)
                        routes = self.compute_routes(msg, b10, b11, rx10, ry10, rx11, ry11)
                        self.print_map(msg, routes)
                        self.process_fsm(msg, routes)"""
                        
    run_loop_new = """                        self.process_ui()
                        if self.mode == "IDLE":
                            self.print_map(msg, {10: None, 11: None})
                            continue
                            
                        b10, b11, rx10, ry10, rx11, ry11 = self.compute_assignment(msg)
                        
                        # Filtrar segun modo
                        if "10" in self.mode: b11 = -1
                        if "11" in self.mode: b10 = -1
                        
                        routes = self.compute_routes(msg, b10, b11, rx10, ry10, rx11, ry11)
                        self.print_map(msg, routes)
                        
                        if "EXEC" in self.mode or self.mode == "AUTO":
                            self.process_fsm(msg, routes)"""
                            
    content = content.replace(run_loop_old, run_loop_new)
    
    # 4. Print current mode in map
    map_footer_old = """        out += "+" + "--"*map_w + "+\n"
        out += "ROVER 10: {} | ROVER 11: {}\\n".format(self.states[10], self.states[11])"""
        
    map_footer_new = """        out += "+" + "--"*map_w + "+\n"
        out += "MODO ACTUAL: {}\\n".format(self.mode)
        out += "ROVER 10: {} | ROVER 11: {}\\n".format(self.states[10], self.states[11])"""
        
    content = content.replace(map_footer_old, map_footer_new)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
