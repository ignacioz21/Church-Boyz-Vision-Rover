import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Hacer que el rover empuje hasta el fondo del deposito (dist < 0.5f en lugar de 2.0f)
    old_push = """        } else if (state == DRIVE_PUSH) {
            if (dist < 2.0f) {"""
            
    new_push = """        } else if (state == DRIVE_PUSH) {
            if (dist < 0.5f) {"""
            
    content = content.replace(old_push, new_push)

    # 2. Reducir la espera de verificacion de 1500ms a 500ms para que no se quede "esperando" tanto
    old_wait = """            if (millis() - wait_start_ms > 1500) {"""
    new_wait = """            if (millis() - wait_start_ms > 400) {"""
    content = content.replace(old_wait, new_wait)

    with open(filepath, 'w') as f:
        f.write(content)

for rid in [1, 2]:
    patch(f"rover_qa/rover_{rid}/src/navigation.cpp")
print("Push tolerance and wait time patched")
