import re

def strip_ino(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Remove simulateRoutesTask from cmd == '8'
    content = content.replace("simulateRoutesTask();", "Serial.println(\"Comando 8 delegado al Cerebro Python.\");")
    
    # Remove printMap(snap) calls
    content = content.replace("printMap(snap);", "// printMap(snap); // Dibujo delegado a Python")

    with open(filepath, 'w') as f:
        f.write(content)

strip_ino("rover_qa/rover_1/rover_1.ino")
strip_ino("rover_qa/rover_2/rover_2.ino")
