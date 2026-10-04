import os

def fix_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_r10 = """        Vector2D pre_app = calculatePreApproach(c_pos, d_pos, getPreApproachDistance());
        sim_r10_p2_x = pre_app.x; sim_r10_p2_y = pre_app.y;
        sim_r10_p3_x = d_pos.x; sim_r10_p3_y = d_pos.y;"""
        
    new_r10 = """        float dx = c_pos.x - d_pos.x;
        float dy = c_pos.y - d_pos.y;
        float len = sqrt(dx*dx + dy*dy);
        if (len == 0.0f) len = 1.0f;
        float pre_x = c_pos.x + (dx/len) * getPreApproachDistance();
        float pre_y = c_pos.y + (dy/len) * getPreApproachDistance();
        pre_x = constrain(pre_x, 3.0f, snap.grid_cols - 3.0f);
        pre_y = constrain(pre_y, 3.0f, snap.grid_rows - 3.0f);
        
        sim_r10_p2_x = pre_x; sim_r10_p2_y = pre_y;
        sim_r10_p3_x = d_pos.x; sim_r10_p3_y = d_pos.y;"""

    old_r11 = """        Vector2D pre_app = calculatePreApproach(c_pos, d_pos, getPreApproachDistance());
        sim_r11_p2_x = pre_app.x; sim_r11_p2_y = pre_app.y;
        sim_r11_p3_x = d_pos.x; sim_r11_p3_y = d_pos.y;"""
        
    new_r11 = """        float dx = c_pos.x - d_pos.x;
        float dy = c_pos.y - d_pos.y;
        float len = sqrt(dx*dx + dy*dy);
        if (len == 0.0f) len = 1.0f;
        float pre_x = c_pos.x + (dx/len) * getPreApproachDistance();
        float pre_y = c_pos.y + (dy/len) * getPreApproachDistance();
        pre_x = constrain(pre_x, 3.0f, snap.grid_cols - 3.0f);
        pre_y = constrain(pre_y, 3.0f, snap.grid_rows - 3.0f);
        
        sim_r11_p2_x = pre_x; sim_r11_p2_y = pre_y;
        sim_r11_p3_x = d_pos.x; sim_r11_p3_y = d_pos.y;"""

    content = content.replace(old_r10, new_r10, 1)
    content = content.replace(old_r11, new_r11, 1)

    with open(filepath, 'w') as f:
        f.write(content)

fix_file("rover_qa/rover_1/src/navigation.cpp")
fix_file("rover_qa/rover_2/src/navigation.cpp")
