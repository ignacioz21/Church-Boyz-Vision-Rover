import os

files = [
    'rover_qa/rover_1/src/hardware.cpp',
    'rover_qa/rover_2/src/hardware.cpp'
]

for filepath in files:
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Let's filter out all lines containing getMotorTrimL or getMotorTrimR
    cleaned_lines = []
    for line in lines:
        if "getMotorTrimL" not in line and "getMotorTrimR" not in line:
            cleaned_lines.append(line)
            
    # Now insert them safely right after setMotorTrim block
    content = "".join(cleaned_lines)
    find_str = """void setMotorTrim(float left, float right) {
    current_trim_l = left;
    current_trim_r = right;
}"""
    replace_str = """void setMotorTrim(float left, float right) {
    current_trim_l = left;
    current_trim_r = right;
}
float getMotorTrimL() { return current_trim_l; }
float getMotorTrimR() { return current_trim_r; }"""

    content = content.replace(find_str, replace_str)
    
    with open(filepath, 'w') as f:
        f.write(content)

print("Fixed.")
