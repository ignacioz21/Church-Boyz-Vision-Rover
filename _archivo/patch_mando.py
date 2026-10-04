import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Update menu text
    menu_old = """print("--- MODO RETO ---")
print("  7 : INICIAR RETO GLOBAL (Ambos rovers se coordinan)")
print("  8 : Imprimir Rutas (Simulacion del Modo Reto en Mapa UDP)")"""

    menu_new = """print("--- CEREBRO DIGITAL (central_brain.py) ---")
print("  t1 : Trazar ruta (Solo Rover 10)")
print("  e1 : Ejecutar ruta (Solo Rover 10)")
print("  t2 : Trazar ruta (Solo Rover 11)")
print("  e2 : Ejecutar ruta (Solo Rover 11)")
print("  tt : Trazar rutas (Ambos rovers)")
print("  ee : Ejecutar rutas (Ambos rovers - AUTO)")
print("  0  : IDLE (Detener todo)")
print("--- COMANDOS MANUALES ESP32 ---")"""

    content = content.replace(menu_old, menu_new)

    # 2. Add Brain dispatching
    dispatch_old = """            # Si el comando contiene dos puntos (ej. 10:6), mandamos todo el string. Si no, solo el primer caracter
            c = cmd.strip().encode() if ':' in cmd else cmd.strip()[0].encode()
            for d in destinos:
                try:
                    sock.sendto(c, (d, 8889))
                except Exception:
                    pass"""
                    
    dispatch_new = """            cmd_str = cmd.strip().lower()
            brain_cmd = None
            if cmd_str == 't1': brain_cmd = b"B:PLAN_10"
            elif cmd_str == 'e1': brain_cmd = b"B:EXEC_10"
            elif cmd_str == 't2': brain_cmd = b"B:PLAN_11"
            elif cmd_str == 'e2': brain_cmd = b"B:EXEC_11"
            elif cmd_str == 'tt': brain_cmd = b"B:PLAN_ALL"
            elif cmd_str == 'ee': brain_cmd = b"B:AUTO"
            elif cmd_str == '0': brain_cmd = b"B:IDLE"
            
            if brain_cmd:
                try: sock.sendto(brain_cmd, ("127.0.0.1", 8890))
                except Exception: pass
                print(f"-> Comando {brain_cmd} enviado al Cerebro Digital.")
                continue

            # Comandos normales ESP32
            c = cmd.strip().encode() if ':' in cmd else cmd.strip()[0].encode()
            for d in destinos:
                try:
                    sock.sendto(c, (d, 8889))
                except Exception:
                    pass"""
                    
    content = content.replace(dispatch_old, dispatch_new)

    with open(filepath, 'w') as f:
        f.write(content)

patch("mando.py")
