import os

files = [
    'rover_qa/rover_1/rover_1.ino',
    'rover_qa/rover_2/rover_2.ino'
]

for filepath in files:
    with open(filepath, 'r') as f: content = f.read()
    
    find_str = """        str_cmd.trim();
        if (str_cmd.length() >= 1) {
            cmd = str_cmd.charAt(0);
        }"""
        
    replace_str = """        str_cmd.trim();
        
        // Soporte para comandos dirigidos a un ID especifico (ej. "10:6")
        if (str_cmd.indexOf(":") != -1 && str_cmd.charAt(0) != 'D') {
            int sep = str_cmd.indexOf(":");
            int target_id = str_cmd.substring(0, sep).toInt();
            if (target_id != telemetryGetRoverId()) {
                cmd = '\\0'; // Ignorar paquete, es para el otro rover
                str_cmd = "";
            } else {
                str_cmd = str_cmd.substring(sep + 1);
            }
        }
        
        if (str_cmd.length() >= 1) {
            cmd = str_cmd.charAt(0);
        }"""
        
    if "Soporte para comandos dirigidos" not in content:
        content = content.replace(find_str, replace_str)
        with open(filepath, 'w') as f: f.write(content)

print("Targeted commands patched in INOs.")
