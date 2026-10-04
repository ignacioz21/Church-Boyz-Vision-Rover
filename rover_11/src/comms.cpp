#include "../include/comms.h"
#include "../include/config.h"
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP udp;
static bool started = false;

static bool ensureStarted() {
    if (WiFi.status() != WL_CONNECTED) {
        started = false;
        return false;
    }
    if (!started) started = udp.begin(ROVER_CMD_PORT);
    return started;
}

bool commsPoll(String &cmd) {
    if (!ensureStarted()) return false;

    // Se procesan todos los paquetes pendientes y se devuelve el último dirigido
    // a este rover: un comando viejo nunca se ejecuta después de uno nuevo.
    bool got = false;
    while (udp.parsePacket() > 0) {
        char buf[96];
        int len = udp.read(buf, sizeof(buf) - 1);
        if (len <= 0) continue;
        buf[len] = '\0';
        String msg(buf);
        msg.trim();

        int sep = msg.indexOf(':');
        if (sep <= 0) continue;
        String target = msg.substring(0, sep);
        if (target != "*" && target.toInt() != ROVER_ID) continue;

        cmd = msg.substring(sep + 1);
        got = true;
    }
    return got;
}

void commsSend(const char *line) {
    if (!ensureStarted()) return;
    udp.beginPacket(BRAIN_HOST, BRAIN_STATUS_PORT);
    udp.write((const uint8_t*)line, strlen(line));
    udp.endPacket();
}
