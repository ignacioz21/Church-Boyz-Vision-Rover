import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_run = """                        if "EXEC" in self.mode or self.mode == "AUTO":
                            self.process_hybrid(msg)"""
                            
    new_run = """                        if "EXEC" in self.mode or self.mode == "AUTO":
                            self.process_fsm(msg, routes)"""
                            
    content = content.replace(old_run, new_run)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Switched to FULL PYTHON FSM control")
