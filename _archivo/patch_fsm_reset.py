import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_reset = """            # Reset state if assignment changed, PERO no interrumpir el Backup
            if state not in ["DRIVE_BACKUP", "TURN_HOME", "DRIVE_HOME", "PARKED"]:
                if current_target != self.last_targets[rid]:
                    if current_target != -1:
                        self.states[rid] = "TURN_PRE"
                    else:
                        self.states[rid] = "TURN_HOME"
                    self.last_targets[rid] = current_target
                    state = self.states[rid]"""
                    
    new_reset = """            # Reset state if assignment changed, PERO no interrumpir el Backup
            if state not in ["DRIVE_BACKUP", "TURN_HOME", "DRIVE_HOME", "PARKED"]:
                # Check if color actually changed
                target_changed = True
                curr_color = route.get("color") if route else None
                last_col_attr = f"last_color_{rid}"
                last_color = getattr(self, last_col_attr, None)
                
                if current_target != -1 and self.last_targets[rid] != -1:
                    if curr_color and curr_color == last_color:
                        target_changed = False
                        
                if curr_color:
                    setattr(self, last_col_attr, curr_color)
                    
                if current_target != self.last_targets[rid] and target_changed:
                    if current_target != -1:
                        self.states[rid] = "TURN_PRE"
                    else:
                        self.states[rid] = "TURN_HOME"
                    state = self.states[rid]
                self.last_targets[rid] = current_target"""
                
    content = content.replace(old_reset, new_reset)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Python FSM reset fixed")
