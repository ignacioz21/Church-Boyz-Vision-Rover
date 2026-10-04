import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Fix push coordinates (was stopping OUTSIDE the depot due to + sign)
    old_push = """            push_x = d["col"] + (dx/length) * 4.0
            push_y = d["row"] + (dy/length) * 4.0"""
            
    new_push = """            # El objetivo de push debe estar MAS ALLA del centro del deposito (signo negativo)
            push_x = d["col"] - (dx/length) * 3.0
            push_y = d["row"] - (dy/length) * 3.0"""
            
    content = content.replace(old_push, new_push)

    # 2. Add proportional speed reduction (deceleration) for distance
    old_drive = """                if arrived:
                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": 
                        self.states[rid] = "DONE"
                        self.last_targets[rid] = -1 # Force re-assignment next frame
                else:
                    fwd_speed = 0.4 # Velocidad base
                    corr = diff * 0.015
                    corr = max(-0.2, min(0.2, corr))
                    ml = fwd_speed - corr
                    mr = fwd_speed + corr"""
                    
    new_drive = """                if arrived:
                    if state == "DRIVE_PRE": self.states[rid] = "TURN_APPROACH"
                    elif state == "SENSOR_APPROACH": self.states[rid] = "TURN_PUSH"
                    elif state == "DRIVE_PUSH": 
                        self.states[rid] = "DONE"
                        self.last_targets[rid] = -1 # Force re-assignment next frame
                else:
                    # Velocidad Proporcional: Frena progresivamente cuando se acerca para anular la latencia UDP
                    fwd_speed = 0.5 
                    if dist < 12.0:
                        fwd_speed = max(0.18, 0.5 * (dist / 12.0))
                        
                    # Si esta empujando el cubo al final, damos un pequeño empuje constante para vencer friccion
                    if state == "DRIVE_PUSH" and dist < 6.0:
                        fwd_speed = max(0.25, fwd_speed)
                        
                    corr = diff * 0.015
                    corr = max(-0.2, min(0.2, corr))
                    ml = fwd_speed - corr
                    mr = fwd_speed + corr"""

    content = content.replace(old_drive, new_drive)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
