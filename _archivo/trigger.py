import socketio
sio = socketio.Client()
sio.connect('http://127.0.0.1:8891')
sio.emit('command', {'cmd': 'EXEC_10'})
sio.disconnect()
