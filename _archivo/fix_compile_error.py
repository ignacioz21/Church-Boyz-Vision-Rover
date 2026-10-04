import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        text = f.read()

    # Find the patrol_border function
    start_idx = text.find('void patrol_border()')
    end_idx = text.find('bool calculateAvoidanceWaypoint', start_idx)

    patrol_str = text[start_idx:end_idx]
    
    # Replace the traffic control block inside patrol_border
    old_traffic = """        if (snap.peer_rover.detected && state != DRIVE_PUSH && state != TURN_PUSH) {"""
    new_traffic = """        if (snap.peer_rover.detected) {"""
    
    patrol_str = patrol_str.replace(old_traffic, new_traffic)
    
    text = text[:start_idx] + patrol_str + text[end_idx:]

    with open(filepath, 'w') as f:
        f.write(text)

fix_file('rover_qa/rover_1/src/navigation.cpp')
fix_file('rover_qa/rover_2/src/navigation.cpp')
