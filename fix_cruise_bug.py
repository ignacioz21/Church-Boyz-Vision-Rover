import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        text = f.read()

    # The buggy string:
    bad_init = "float current_cruise = current_cruise;"
    good_init = "float current_cruise = QA_SPEED_CRUISE;"
    
    bad_limit = "current_cruise = current_cruise * 0.6f;"
    good_limit = "current_cruise = QA_SPEED_CRUISE * 0.6f;"

    text = text.replace(bad_init, good_init)
    text = text.replace(bad_limit, good_limit)

    with open(filepath, 'w') as f:
        f.write(text)

fix_file('rover_qa/rover_1/src/navigation.cpp')
fix_file('rover_qa/rover_2/src/navigation.cpp')
