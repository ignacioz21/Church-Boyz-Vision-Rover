import os

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Fix the tolerance in isOnLineSegment
    old_tol = "return sqrt(pow(px - proj_x, 2) + pow(py - proj_y, 2)) <= 1.0f; // Tolerancia 1 celda"
    new_tol = "return sqrt(pow(px - proj_x, 2) + pow(py - proj_y, 2)) <= 0.6f; // Tolerancia ajustada para diagonales finas"
    content = content.replace(old_tol, new_tol)

    # 2. Fix the 999.0f infinity routing
    old_r10_assign = "sim_r10_p1_x = rx_10; sim_r10_p1_y = ry_10;"
    new_r10_assign = "sim_r10_p1_x = (rx_10 > 900.0f) ? -1.0f : rx_10; sim_r10_p1_y = (ry_10 > 900.0f) ? -1.0f : ry_10;"
    content = content.replace(old_r10_assign, new_r10_assign)

    old_r11_assign = "sim_r11_p1_x = rx_11; sim_r11_p1_y = ry_11;"
    new_r11_assign = "sim_r11_p1_x = (rx_11 > 900.0f) ? -1.0f : rx_11; sim_r11_p1_y = (ry_11 > 900.0f) ? -1.0f : ry_11;"
    content = content.replace(old_r11_assign, new_r11_assign)

    with open(filepath, 'w') as f:
        f.write(content)

fix_file("rover_qa/rover_1/src/navigation.cpp")
fix_file("rover_qa/rover_2/src/navigation.cpp")
