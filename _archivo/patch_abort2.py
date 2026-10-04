import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    old_abort = """bool checkAbort(WiFiUDP &udpCmd) {
    if (udpCmd.parsePacket()) {
        char buf[32];
        int len = udpCmd.read(buf, 31);
        if (len > 0) {
            buf[len] = 0;
            String s = String(buf);
            s.trim();
            char cmd = s.charAt(0);
            if (s.indexOf(':') != -1) {
                int tid = s.substring(0, s.indexOf(':')).toInt();
                if (tid != telemetryGetRoverId()) return false;
                cmd = s.substring(s.indexOf(':') + 1).charAt(0);
            }
            if (cmd == 'r' || cmd == 'R' || cmd == 'h' || cmd == 'H') {
                stopMotors();
                return true;
            }
        }
    }
    return false;
}"""
    
    new_abort = """bool checkAbort(WiFiUDP &udpCmd) {
    bool abort_flag = false;
    // Consumir TODOS los paquetes pendientes en el buffer para evitar LAG
    while (udpCmd.parsePacket()) {
        char buf[32];
        int len = udpCmd.read(buf, 31);
        if (len > 0) {
            buf[len] = 0;
            String s = String(buf);
            s.trim();
            char cmd = s.charAt(0);
            if (s.indexOf(':') != -1) {
                int tid = s.substring(0, s.indexOf(':')).toInt();
                if (tid == telemetryGetRoverId()) {
                    cmd = s.substring(s.indexOf(':') + 1).charAt(0);
                    if (cmd == 'r' || cmd == 'R' || cmd == 'h' || cmd == 'H') {
                        abort_flag = true;
                    }
                }
            } else {
                if (cmd == 'r' || cmd == 'R' || cmd == 'h' || cmd == 'H') {
                    abort_flag = true;
                }
            }
        }
    }
    if (abort_flag) {
        stopMotors();
        return true;
    }
    return false;
}"""
    
    content = content.replace(old_abort, new_abort)

    with open(filepath, 'w') as f:
        f.write(content)

for rid in [1, 2]:
    patch(f"rover_qa/rover_{rid}/src/navigation.cpp")
print("Zero-lag checkAbort patched")
