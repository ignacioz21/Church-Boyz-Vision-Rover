import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    html_btns_old = """                    <button class="danger" onclick="sendCommand('IDLE')">IDLE / RESET CEREBRO (0)</button>
                    <button class="danger" onclick="sendVisionCommand('abort')">ABORTAR VISION</button>
                    <button class="action" onclick="sendVisionCommand('ready')">READY VISION</button>
                    <button class="action" onclick="sendVisionCommand('stop')">STOP VISION</button>"""
                    
    html_btns_new = """                    <button class="danger" onclick="sendCommand('IDLE')">IDLE / PARAR ROVERS (0)</button>
                    <button class="danger" onclick="sendVisionCommand('abort')">ABORTAR VISION</button>
                    <button class="action" onclick="sendVisionCommand('ready')">READY VISION</button>
                    <button class="action" onclick="sendVisionCommand('stop')">STOP VISION</button>
                    
                    <!-- Botones de Sistema -->
                    <button class="success" style="background: #e65100; margin-top: 15px;" onclick="sendCommand('RESTART_BRAIN')">⚡ ENCENDER CEREBRO</button>
                    <button class="success" style="background: #e65100; margin-top: 15px;" onclick="sendCommand('RESTART_VISION')">📷 ENCENDER VISIÓN</button>"""
                    
    content = content.replace(html_btns_old, html_btns_new)

    with open(filepath, 'w') as f:
        f.write(content)

patch("dashboard/index.html")
print("Restart buttons added to UI")
