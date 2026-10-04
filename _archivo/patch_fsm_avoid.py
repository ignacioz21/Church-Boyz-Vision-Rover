import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_target = """            elif route:
                target_x, target_y = route["p2"][0], route["p2"][1] # Default: Pre-approach
                if state in ["TURN_APPROACH", "SENSOR_APPROACH"]:
                    target_x, target_y = route["cube"][0], route["cube"][1]
                elif state in ["TURN_PUSH", "DRIVE_PUSH", "DRIVE_BACKUP"]:
                    target_x, target_y = route["push"][0], route["push"][1]"""
                    
    new_target = """            elif route:
                target_x, target_y = route["p2"][0], route["p2"][1] # Default: Pre-approach
                
                # Dinamica de evasion: Si hay punto de desvio y no hemos llegado, ir alla primero
                if route.get("p_avoid") and state in ["TURN_PRE", "DRIVE_PRE"]:
                    dist_avoid = math.sqrt((route["p_avoid"][0]-x)**2 + (route["p_avoid"][1]-y)**2)
                    if dist_avoid > 3.0:
                        target_x, target_y = route["p_avoid"][0], route["p_avoid"][1]
                        
                if state in ["TURN_APPROACH", "SENSOR_APPROACH"]:
                    target_x, target_y = route["cube"][0], route["cube"][1]
                elif state in ["TURN_PUSH", "DRIVE_PUSH", "DRIVE_BACKUP"]:
                    target_x, target_y = route["push"][0], route["push"][1]"""
                    
    content = content.replace(old_target, new_target)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Python FSM patched for avoidance")
