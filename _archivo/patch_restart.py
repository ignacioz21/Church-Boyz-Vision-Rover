import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Add subprocess import
    if "import subprocess" not in content:
        content = "import subprocess\nimport os\n" + content

    old_cmd = """@socketio.on('ui_command')
def handle_command(data):
    if not 'brain_instance' in globals(): return
    cmd = data.get('cmd')
    if cmd == 'IDLE':"""
    
    new_cmd = """@socketio.on('ui_command')
def handle_command(data):
    global brain_thread, brain_instance
    if not 'brain_instance' in globals(): return
    cmd = data.get('cmd')
    
    if cmd == 'RESTART_VISION':
        web_log("SYS", "Reiniciando Sistema de Visión...")
        os.system("pkill -f 'python -m vision.sistema'")
        subprocess.Popen(["python", "-m", "vision.sistema"])
        return
        
    elif cmd == 'RESTART_BRAIN':
        web_log("SYS", "Reiniciando hilo del Cerebro...")
        if 'brain_thread' in globals() and brain_thread.is_alive():
            web_log("ERR", "El cerebro ya esta corriendo.")
        else:
            brain_instance = CentralBrain()
            brain_thread = threading.Thread(target=brain_instance.run, daemon=True)
            brain_thread.start()
            web_log("SYS", "Cerebro reiniciado exitosamente.")
        return

    if cmd == 'IDLE':"""
    
    content = content.replace(old_cmd, new_cmd)

    # Make sure we store brain_thread globally
    old_main = """    # Run the brain in a daemon thread
    t = threading.Thread(target=brain_instance.run, daemon=True)
    t.start()"""
    
    new_main = """    # Run the brain in a daemon thread
    global brain_thread
    brain_thread = threading.Thread(target=brain_instance.run, daemon=True)
    brain_thread.start()"""
    
    content = content.replace(old_main, new_main)

    with open(filepath, 'w') as f:
        f.write(content)

patch("central_brain.py")
print("Restart logic added to central_brain")
