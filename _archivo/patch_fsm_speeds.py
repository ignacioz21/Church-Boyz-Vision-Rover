import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_speeds = """                    turn_speed = diff * 0.004
                    if 0 < turn_speed < 0.16: turn_speed = 0.16
                    if 0 > turn_speed > -0.16: turn_speed = -0.16
                    turn_speed = max(-0.25, min(0.25, turn_speed)) # Pivot max speed reducido drasticamente"""
                    
    new_speeds = """                    turn_speed = diff * 0.008
                    if 0 < turn_speed < 0.30: turn_speed = 0.30
                    if 0 > turn_speed > -0.30: turn_speed = -0.30
                    turn_speed = max(-0.45, min(0.45, turn_speed)) # Pivot max speed aumentado para vencer friccion"""
                    
    content = content.replace(old_speeds, new_speeds)
    
    # Also fix drive speed min/max if needed
    old_drive = """                    fwd_speed = 0.4 # Velocidad base
                    corr = diff * 0.015
                    corr = max(-0.2, min(0.2, corr))"""
                    
    new_drive = """                    fwd_speed = 0.6 # Velocidad base aumentada para mejor respuesta
                    corr = diff * 0.025
                    corr = max(-0.3, min(0.3, corr))"""
                    
    content = content.replace(old_drive, new_drive)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Python FSM speeds increased")
