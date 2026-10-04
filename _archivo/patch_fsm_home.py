import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_elif = """            elif state in ["DRIVE_PRE", "SENSOR_APPROACH", "DRIVE_PUSH"]:"""
    new_elif = """            elif state in ["DRIVE_PRE", "SENSOR_APPROACH", "DRIVE_PUSH", "DRIVE_HOME"]:"""
    content = content.replace(old_elif, new_elif)

    old_arrived = """                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": 
                        self.states[rid] = "DONE"
                        self.last_targets[rid] = -1 # Force re-assignment next frame"""
                        
    new_arrived = """                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": 
                        self.states[rid] = "DONE"
                        self.last_targets[rid] = -1 # Force re-assignment next frame
                    elif state == "DRIVE_HOME":
                        self.states[rid] = "PARKED" """

    content = content.replace(old_arrived, new_arrived)
    
    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("DRIVE_HOME fixed")
