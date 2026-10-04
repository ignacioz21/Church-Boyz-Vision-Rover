import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Slow down the turning
    old_turn = """                    turn_speed = diff * 0.008
                    if 0 < turn_speed < 0.15: turn_speed = 0.15
                    if 0 > turn_speed > -0.15: turn_speed = -0.15
                    turn_speed = max(-0.4, min(0.4, turn_speed)) # Pivot max speed"""
                    
    new_turn = """                    turn_speed = diff * 0.004
                    if 0 < turn_speed < 0.16: turn_speed = 0.16
                    if 0 > turn_speed > -0.16: turn_speed = -0.16
                    turn_speed = max(-0.25, min(0.25, turn_speed)) # Pivot max speed reducido drasticamente"""
                    
    content = content.replace(old_turn, new_turn)

    # 2. Slow down the forward driving
    old_drive = """                    # Velocidad Proporcional: Frena progresivamente cuando se acerca para anular la latencia UDP
                    fwd_speed = 0.5 
                    if dist < 12.0:
                        fwd_speed = max(0.18, 0.5 * (dist / 12.0))
                        
                    # Si esta empujando el cubo al final, damos un pequeño empuje constante para vencer friccion
                    if state == "DRIVE_PUSH" and dist < 6.0:
                        fwd_speed = max(0.25, fwd_speed)
                        
                    corr = diff * 0.015
                    corr = max(-0.2, min(0.2, corr))"""
                    
    new_drive = """                    # Velocidad Proporcional ultra-lenta
                    fwd_speed = 0.25 
                    if dist < 12.0:
                        fwd_speed = max(0.16, 0.25 * (dist / 12.0))
                        
                    if state == "DRIVE_PUSH" and dist < 6.0:
                        fwd_speed = max(0.20, fwd_speed)
                        
                    corr = diff * 0.008
                    corr = max(-0.15, min(0.15, corr))"""
                    
    content = content.replace(old_drive, new_drive)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
