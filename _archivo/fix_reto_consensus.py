import sys, os

files = [
    'rover_qa/rover_1/src/navigation.cpp',
    'rover_qa/rover_2/src/navigation.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    find_str = """    float rx_me = snap.my_rover.x;
    float ry_me = snap.my_rover.y;
    float rx_peer = snap.peer_rover.detected ? snap.peer_rover.x : 999.0f; // Si no lo vemos, asumimos lejos
    float ry_peer = snap.peer_rover.detected ? snap.peer_rover.y : 999.0f;
    
    for (int step = 0; step < 3; step++) {
        float min_dist = 99999.0f;
        int best_c = -1;
        int best_rover = -1;
        
        for (int c = 0; c < 3; c++) {
            if (cube_assigned[c] || !snap.cubes[c].detected) continue;
            
            // Distancia a MI
            float d_me = sqrt(pow(snap.cubes[c].x - rx_me, 2) + pow(snap.cubes[c].y - ry_me, 2));
            if (d_me < min_dist) {
                min_dist = d_me;
                best_c = c;
                best_rover = my_id;
            }
            
            // Distancia al PEER
            float d_peer = sqrt(pow(snap.cubes[c].x - rx_peer, 2) + pow(snap.cubes[c].y - ry_peer, 2));
            if (d_peer < min_dist) {
                min_dist = d_peer;
                best_c = c;
                best_rover = peer_id;
            }
        }"""
        
    replace_str = """    float rx_10, ry_10, rx_11, ry_11;
    if (my_id == 10) {
        rx_10 = snap.my_rover.x; ry_10 = snap.my_rover.y;
        rx_11 = snap.peer_rover.detected ? snap.peer_rover.x : 999.0f;
        ry_11 = snap.peer_rover.detected ? snap.peer_rover.y : 999.0f;
    } else {
        rx_11 = snap.my_rover.x; ry_11 = snap.my_rover.y;
        rx_10 = snap.peer_rover.detected ? snap.peer_rover.x : 999.0f;
        ry_10 = snap.peer_rover.detected ? snap.peer_rover.y : 999.0f;
    }
    
    for (int step = 0; step < 3; step++) {
        float min_dist = 99999.0f;
        int best_c = -1;
        int best_rover = -1;
        
        for (int c = 0; c < 3; c++) {
            if (cube_assigned[c] || !snap.cubes[c].detected) continue;
            
            // Evaluamos SIEMPRE en el mismo orden absoluto (ID 10 primero) para evitar empates asimétricos
            float d_10 = sqrt(pow(snap.cubes[c].x - rx_10, 2) + pow(snap.cubes[c].y - ry_10, 2));
            if (d_10 < min_dist) {
                min_dist = d_10;
                best_c = c;
                best_rover = 10;
            }
            
            float d_11 = sqrt(pow(snap.cubes[c].x - rx_11, 2) + pow(snap.cubes[c].y - ry_11, 2));
            if (d_11 < min_dist) {
                min_dist = d_11;
                best_c = c;
                best_rover = 11;
            }
        }"""
        
    content = content.replace(find_str, replace_str)
    
    with open(filepath, 'w') as f: f.write(content)

print("Consensus fixed.")
