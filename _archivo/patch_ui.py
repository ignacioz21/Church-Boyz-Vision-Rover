import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # 1. Light Theme CSS changes
    css_old = """        body { margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: #ffffff; display: flex; flex-direction: column; height: 100vh; overflow: hidden; }
        .header { background: #1f1f1f; padding: 10px 20px; display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid #333; }
        .header h1 { margin: 0; font-size: 1.5rem; color: #4CAF50; }
        .status-bar { font-size: 0.9rem; color: #bbb; }
        
        .main-container { display: flex; flex: 1; overflow: hidden; }
        
        /* Left Column */
        .left-panel { flex: 1; display: flex; flex-direction: column; border-right: 2px solid #333; background: #000; position: relative; }
        .video-container { flex: 1; display: flex; justify-content: center; align-items: center; overflow: hidden; position: relative; }
        .video-container img { max-width: 100%; max-height: 100%; object-fit: contain; }
        .video-overlay { position: absolute; top: 10px; left: 10px; background: rgba(0,0,0,0.6); padding: 5px 10px; border-radius: 5px; font-family: monospace; }
        
        .vision-logs { height: 200px; background: #1e1e1e; border-top: 2px solid #333; padding: 10px; overflow-y: auto; font-family: monospace; font-size: 12px; }
        
        /* Right Column */
        .right-panel { width: 500px; display: flex; flex-direction: column; background: #1a1a1a; }
        
        .map-container { height: 350px; background: #000; border-bottom: 2px solid #333; position: relative; display: flex; justify-content: center; align-items: center; }"""
        
    css_new = """        body { margin: 0; padding: 0; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #f5f5f5; color: #333; display: flex; flex-direction: column; height: 100vh; overflow: hidden; }
        .header { background: #fff; padding: 10px 20px; display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #ccc; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .header h1 { margin: 0; font-size: 1.5rem; color: #2e7d32; }
        .status-bar { font-size: 0.9rem; color: #666; font-weight: bold; }
        
        .main-container { display: flex; flex: 1; overflow: hidden; }
        
        /* Left Column */
        .left-panel { flex: 1; display: flex; flex-direction: column; border-right: 1px solid #ccc; background: #fafafa; position: relative; }
        .video-container { flex: 1; display: flex; justify-content: center; align-items: center; overflow: hidden; position: relative; background: #e0e0e0; }
        .video-container img { max-width: 100%; max-height: 100%; object-fit: contain; }
        .video-overlay { position: absolute; top: 10px; left: 10px; background: rgba(255,255,255,0.8); color: #000; padding: 5px 10px; border-radius: 5px; font-family: monospace; font-weight: bold; border: 1px solid #ccc; }
        
        .vision-logs { height: 200px; background: #fff; border-top: 1px solid #ccc; padding: 10px; overflow-y: auto; font-family: monospace; font-size: 12px; }
        
        /* Right Column */
        .right-panel { width: 500px; display: flex; flex-direction: column; background: #fff; }
        
        .map-container { height: 350px; background: #eee; border-bottom: 1px solid #ccc; position: relative; display: flex; justify-content: center; align-items: center; }"""
        
    content = content.replace(css_old, css_new)
    
    css_logs_old = """        .brain-logs { flex: 1; background: #0f0f0f; padding: 10px; overflow-y: auto; font-family: monospace; font-size: 13px; }
        
        /* Log Tag Colors */
        .log-line { margin: 2px 0; border-bottom: 1px solid #222; padding-bottom: 2px; }
        .tag-tx { color: #fbc02d; font-weight: bold; }
        .tag-rx { color: #29b6f6; font-weight: bold; }
        .tag-brain { color: #66bb6a; font-weight: bold; }
        .tag-err { color: #ef5350; font-weight: bold; }
        .tag-vision { color: #ab47bc; font-weight: bold; }"""
        
    css_logs_new = """        .brain-logs { flex: 1; background: #f9f9f9; padding: 10px; overflow-y: auto; font-family: monospace; font-size: 13px; border-top: 1px solid #eee; }
        
        /* Log Tag Colors */
        .log-line { margin: 2px 0; border-bottom: 1px solid #eee; padding-bottom: 2px; }
        .tag-tx { color: #f57f17; font-weight: bold; }
        .tag-rx { color: #0277bd; font-weight: bold; }
        .tag-brain { color: #2e7d32; font-weight: bold; }
        .tag-err { color: #c62828; font-weight: bold; }
        .tag-vision { color: #6a1b9a; font-weight: bold; }"""
        
    content = content.replace(css_logs_old, css_logs_new)

    # 2. Add buttons and fix video src
    html_old = """        <!-- Panel Izquierdo: Camara y Logs de Vision -->
        <div class="left-panel">
            <div class="video-container">
                <div class="video-overlay">CAMARA EN VIVO (8080)</div>
                <!-- Reemplazar IP si se accede desde otra maquina -->
                <img id="videoStream" src="http://localhost:8080/video_feed" alt="Video stream desconectado" onerror="this.alt='Esperando señal de video en puerto 8080...'; this.style.display='none'; setTimeout(()=> { this.src='http://localhost:8080/video_feed?'+new Date().getTime(); this.style.display='block'; }, 2000);">
            </div>"""
            
    html_new = """        <!-- Panel Izquierdo: Camara y Logs de Vision -->
        <div class="left-panel">
            <div class="video-container">
                <div class="video-overlay">CAMARA EN VIVO</div>
                <img id="videoStream" alt="Video stream desconectado" onerror="this.alt='Esperando señal de video en puerto 8080...'; this.style.display='none'; setTimeout(initVideoStream, 2000);">
            </div>"""
            
    content = content.replace(html_old, html_new)
    
    html_btns_old = """                    <button class="action" onclick="sendCommand('EXEC_11')">Ejecutar R11 (e2)</button>
                    <button class="success" onclick="sendCommand('PLAN_ALL')">Trazar Ambos (tt)</button>
                    <button class="success" onclick="sendCommand('AUTO')">Ejecutar AUTO (ee)</button>
                    <button class="danger" style="grid-column: span 2;" onclick="sendCommand('IDLE')">ABORTAR / IDLE (0)</button>"""
                    
    html_btns_new = """                    <button class="action" onclick="sendCommand('EXEC_11')">Ejecutar R11 (e2)</button>
                    <button class="success" onclick="sendCommand('PLAN_ALL')">Trazar Ambos (tt)</button>
                    <button class="success" onclick="sendCommand('AUTO')">Ejecutar AUTO (ee)</button>
                    <button class="danger" onclick="sendCommand('IDLE')">IDLE / RESET CEREBRO (0)</button>
                    <button class="danger" onclick="sendVisionCommand('abort')">ABORTAR VISION</button>
                    <button class="action" onclick="sendVisionCommand('ready')">READY VISION</button>
                    <button class="action" onclick="sendVisionCommand('stop')">STOP VISION</button>"""
                    
    content = content.replace(html_btns_old, html_btns_new)
    
    # 3. JS scripts
    js_old = """        // Conectar al Cerebro (WebSockets)
        const socket = io(); // Connects back to the same host that served the page"""
        
    js_new = """        // Init video stream dynamically
        function initVideoStream() {
            const host = window.location.hostname;
            const video = document.getElementById('videoStream');
            video.src = `http://${host}:8080/video_feed?` + new Date().getTime();
            video.style.display = 'block';
        }
        initVideoStream();
        
        function sendVisionCommand(cmd) {
            const host = window.location.hostname;
            fetch(`http://${host}:8080/command?cmd=${cmd}`)
                .then(r => appendVisionLog('SYS', 'Enviado: ' + cmd))
                .catch(e => appendVisionLog('ERR', 'Error red: ' + cmd));
        }

        // Conectar al Cerebro (WebSockets)
        const socket = io(); // Connects back to the same host that served the page"""
        
    content = content.replace(js_old, js_new)

    with open(filepath, 'w') as f:
        f.write(content)

patch("dashboard/index.html")
print("Dashboard Light Theme and Vision Commands injected")
