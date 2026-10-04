import re

def patch(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # Find checkAbort
    old_abort = """bool checkAbort(WiFiUDP &udpCmd) {
    if (udpCmd.parsePacket() && udpCmd.read() == 'r') {
        udpCmd.flush();
        stopMotors();
        return true;
    }
    return false;
}"""
    
    new_abort = """bool checkAbort(WiFiUDP &udpCmd) {
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
    
    content = content.replace(old_abort, new_abort)

    with open(filepath, 'w') as f:
        f.write(content)

for rid in [1, 2]:
    patch(f"rover_qa/rover_{rid}/src/navigation.cpp")
print("Targeted checkAbort patched")
